#include "config.h"
#include "state_machine.h"
#include "cubic_spline.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>
#include <memory>

class StateBase {
public:
    virtual ~StateBase() {}
    virtual std::unique_ptr<StateBase> execute(StateMachine* sm) = 0;
};

class HoverState : public StateBase {
public:
    std::unique_ptr<StateBase> execute(StateMachine* sm) override {
        ROS_INFO_ONCE("Drone is hovering!");
        return nullptr;
    }
};

class ToCaveState : public StateBase {
public:
    std::unique_ptr<StateBase> execute(StateMachine* sm) override {
        ROS_INFO_ONCE("Drone is flying to cave!");
        if (!sm->goal_sent_once)
        {
            sm->goal_sent_once = true;
            sm->goalpoint = sm->getNextGoalPoint();
            auto goal_pose = sm->pointToPoseStamped(sm->goalpoint, "world", -1.5708 * 2);
            sm->goal_position_pub_.publish(goal_pose);
            ROS_INFO("Published to-cave goal: [%f, %f, %f]",
                     sm->goalpoint.x, sm->goalpoint.y, sm->goalpoint.z);
        }
        if (sm->goal_reached())
        {
            sm->goal_sent_once = false;
            if (sm->current_goal_index == 3)
            {
                sm->yaw_des = -1.5708;
                sm->set_position();
                return std::make_unique<HoverState>();
            }
        }
        return nullptr;
    }
};

class TakeoffState : public StateBase {
public:
    std::unique_ptr<StateBase> execute(StateMachine* sm) override {
        ROS_INFO_ONCE("Drone is taking off!");
        if (!sm->goal_sent_once)
        {
            sm->goal_sent_once = true;
            sm->goalpoint = sm->getNextGoalPoint();
            ros::Duration(1).sleep();
            auto goal_pose = sm->pointToPoseStamped(sm->goalpoint, "world");
            sm->goal_position_pub_.publish(goal_pose);
            ROS_INFO("Published takeoff goal: [%f, %f, %f]",
                     sm->goalpoint.x, sm->goalpoint.y, sm->goalpoint.z);
        }
        if (sm->goal_reached())
        {
            sm->goal_sent_once = false;
            return std::make_unique<ToCaveState>();
        }
        return nullptr;
    }
};

class ForwardState : public StateBase {
public:
    double target_yaw;
    ForwardState(double yaw) : target_yaw(yaw) {}
    std::unique_ptr<StateBase> execute(StateMachine* sm) override {
        ROS_INFO_ONCE("Drone is flying forward!");
        tf::Vector3 pos(sm->cur_position[0], sm->cur_position[1] - 3, sm->cur_position[2]);
        tf::Quaternion q;
        q.setRPY(0, 0, target_yaw);
        sm->set_waypoint(pos, q);
        if (sm->in_range(sm->pos_[1] - config::tol, sm->pos_[1] + config::tol, sm->cur_position[1] - 5))
        {
            sm->set_position();
            return std::make_unique<HoverState>();
        }
        return nullptr;
    }
};

class TurnState : public StateBase {
public:
    double target_yaw;
    TurnState(double yaw) : target_yaw(yaw) {}
    std::unique_ptr<StateBase> execute(StateMachine* sm) override {
        ROS_INFO_ONCE("Drone is turning!");
        tf::Vector3 pos(sm->cur_position[0], sm->cur_position[1], sm->cur_position[2]);
        tf::Quaternion q;
        q.setRPY(0, 0, target_yaw);
        sm->set_waypoint(pos, q);
        if (std::fabs(sm->yaw_ - target_yaw) < 0.01)
        {
            sm->set_position();
            return std::make_unique<ForwardState>(target_yaw);
        }
        return nullptr;
    }
};

class LandingState : public StateBase {
public:
    std::unique_ptr<StateBase> execute(StateMachine* sm) override {
        ROS_INFO_ONCE("Drone is landing!");
        tf::Vector3 pos(sm->cur_position[0], sm->cur_position[1], 0);
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        sm->set_waypoint(pos, q);
        return nullptr;
    }
};

StateMachine::StateMachine()
{
    double x = -38.0, y = 10.0, z = 6.9;

    addGoalPoint(-38.0, 10.0, 10.0);
    addGoalPoint(-55, 0.84, 15.0);
    addGoalPoint(-321, 10.0, 15.0);
    addGoalPoint(-500, 0.0, 10.0);
    addGoalPoint(-599, -9.0, 10.0);
    addGoalPoint(-599, -5, 10.0);
    addGoalPoint(-599, -2, 3.0);

    origin_ = tf::Vector3(x, y, z);

    desired_state_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
    current_state_sub_ = nh.subscribe("/current_state_est", 1, &StateMachine::onCurrentState, this);
    goal_position_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_position", 1);
    path_sub_ = nh.subscribe("planned_path", 1, &StateMachine::onPlannedPath, this);

    state_machine_timer_ = nh.createTimer(ros::Duration(config::sim_interval),
                                          &StateMachine::state_machine_mission, this);

    current_state = std::make_unique<TakeoffState>();
}

geometry_msgs::Point StateMachine::getNextGoalPoint()
{
    if (current_goal_index < goalpoints.size())
        return goalpoints[current_goal_index++];
    else
        return goalpoints.back();
}

geometry_msgs::PoseStamped StateMachine::pointToPoseStamped(const geometry_msgs::Point &point, const std::string &frame_id, double yaw)
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose.position = point;
    tf::Quaternion quat = tf::createQuaternionFromYaw(yaw);
    quaternionTFToMsg(quat, pose_stamped.pose.orientation);
    return pose_stamped;
}

void StateMachine::addGoalPoint(double x, double y, double z)
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    goalpoints.push_back(point);
}

void StateMachine::onCurrentState(const nav_msgs::Odometry &cur_state)
{
    pos_ << cur_state.pose.pose.position.x,
            cur_state.pose.pose.position.y,
            cur_state.pose.pose.position.z;
    vel_ << cur_state.twist.twist.linear.x,
            cur_state.twist.twist.linear.y,
            cur_state.twist.twist.linear.z;
    omega_ << cur_state.twist.twist.angular.x,
              cur_state.twist.twist.angular.y,
              cur_state.twist.twist.angular.z;
    tf::quaternionMsgToTF(cur_state.pose.pose.orientation, quat_);
    yaw_ = tf::getYaw(quat_);
}

bool StateMachine::in_range(double low, double high, double x)
{
    return ((x - high) * (x - low) <= 0);
}

bool StateMachine::goal_reached()
{
    return (in_range(goalpoint.x - config::tol, goalpoint.x + config::tol, pos_[0]) &&
            in_range(goalpoint.y - config::tol, goalpoint.y + config::tol, pos_[1]) &&
            in_range(goalpoint.z - config::tol, goalpoint.z + config::tol, pos_[2]));
}

void StateMachine::set_waypoint(tf::Vector3 pos, tf::Quaternion q,
                                tf::Vector3 lin_vel, tf::Vector3 ang_vel, tf::Vector3 lin_acc)
{
    tf::Transform desired_pos(tf::Transform::getIdentity());
    geometry_msgs::Twist vel;
    geometry_msgs::Twist acc;

    desired_pos.setOrigin(pos);
    desired_pos.setRotation(q);

    vel.linear.x = lin_vel.getX();
    vel.linear.y = lin_vel.getY();
    vel.linear.z = lin_vel.getZ();

    vel.angular.x = ang_vel.getX();
    vel.angular.y = ang_vel.getY();
    vel.angular.z = ang_vel.getZ();

    acc.linear.x = lin_acc.getX();
    acc.linear.y = lin_acc.getY();
    acc.linear.z = lin_acc.getZ();

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
    msg.transforms.resize(1);
    msg.transforms[0].translation.x = desired_pos.getOrigin().x();
    msg.transforms[0].translation.y = desired_pos.getOrigin().y();
    msg.transforms[0].translation.z = desired_pos.getOrigin().z();
    msg.transforms[0].rotation.x = desired_pos.getRotation().getX();
    msg.transforms[0].rotation.y = desired_pos.getRotation().getY();
    msg.transforms[0].rotation.z = desired_pos.getRotation().getZ();
    msg.transforms[0].rotation.w = desired_pos.getRotation().getW();

    msg.velocities.resize(1);
    msg.velocities[0] = vel;
    msg.accelerations.resize(1);
    msg.accelerations[0] = acc;

    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.points.push_back(msg);
    desired_state_pub_.publish(trajectory_msg);

    br.sendTransform(tf::StampedTransform(desired_pos, ros::Time::now(),
                                          "world", "av-desired"));
}

void StateMachine::set_position()
{
    cur_position << pos_[0], pos_[1], pos_[2];
}

void StateMachine::sendGoal(const geometry_msgs::Point &point, const std::string &frame_id, double yaw)
{
    goalpoint = point;
    auto goal_pose_stamped = pointToPoseStamped(point, frame_id, yaw);
    goal_position_pub_.publish(goal_pose_stamped);
}

void StateMachine::updateCurrentPosition()
{
    cur_position << pos_[0], pos_[1], pos_[2];
}

void StateMachine::onPlannedPath(const nav_msgs::Path::ConstPtr &msg)
{
    std::vector<geometry_msgs::PoseStamped> poses = msg->poses;
    if (poses.empty()) return;

    std::vector<double> t, xs, ys, zs, yaws;
    t.push_back(0.0);
    xs.push_back(poses[0].pose.position.x);
    ys.push_back(poses[0].pose.position.y);
    zs.push_back(poses[0].pose.position.z);
    double initial_yaw = (poses.size() > 1) ?
        std::atan2(poses[1].pose.position.y - poses[0].pose.position.y,
                   poses[1].pose.position.x - poses[0].pose.position.x) : 0.0;
    yaws.push_back(initial_yaw);

    for (size_t i = 1; i < poses.size(); i++)
    {
        double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
        double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
        double dz = poses[i].pose.position.z - poses[i-1].pose.position.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        t.push_back(t.back() + dist);
        xs.push_back(poses[i].pose.position.x);
        ys.push_back(poses[i].pose.position.y);
        zs.push_back(poses[i].pose.position.z);
        double yaw;
        if (i < poses.size()-1) {
            yaw = std::atan2(poses[i+1].pose.position.y - poses[i].pose.position.y,
                             poses[i+1].pose.position.x - poses[i].pose.position.x);
        } else {
            if (yaws.size() >= 2) {
                double yaw_diff = yaws.back() - yaws[yaws.size()-2];
                yaw = yaws.back() + yaw_diff;
            } else {
                yaw = yaws.back();
            }
        }
        yaws.push_back(yaw);
    }

    CubicSpline spline_x, spline_y, spline_z, spline_yaw;
    spline_x.set_points(t, xs);
    spline_y.set_points(t, ys);
    spline_z.set_points(t, zs);
    spline_yaw.set_points(t, yaws);

    int num_interp = config::interpolation_points;
    double t_max = t.back();
    double dt = t_max / (num_interp - 1);

    goalpoints_path_vec.clear();
    for (int i = 0; i < num_interp; i++)
    {
        double ti = i * dt;
        double ix = spline_x(ti);
        double iy = spline_y(ti);
        double iz = spline_z(ti);
        double iyaw = spline_yaw(ti);
        goalpoints_path_vec.push_back(Eigen::Vector4d(ix, iy, iz, iyaw));
    }

    if (!goalpoints_path_vec.empty())
    {
        const Eigen::Vector4d &goalPoint = goalpoints_path_vec.back();
        geometry_msgs::PoseStamped goal_pose_stamped;
        goal_pose_stamped.pose.position.x = goalPoint[0];
        goal_pose_stamped.pose.position.y = goalPoint[1];
        goal_pose_stamped.pose.position.z = goalPoint[2];

        geometry_msgs::Quaternion q_msg;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, goalPoint[3]);
        q_msg.x = quat.x();
        q_msg.y = quat.y();
        q_msg.z = quat.z();
        q_msg.w = quat.w();
        goal_pose_stamped.pose.orientation = q_msg;

        goal_pose_stamped.header.frame_id = "world";
        goal_pose_stamped.header.stamp = ros::Time::now();

        goal_position_pub_.publish(goal_pose_stamped);
    }
}

void StateMachine::state_machine_mission(const ros::TimerEvent &t)
{
    if (current_state)
    {
        std::unique_ptr<StateBase> new_state = current_state->execute(this);
        if (new_state)
        {
            current_state = std::move(new_state);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ROS_INFO_ONCE("State machine initialized.");
    StateMachine statemachine;
    ros::spin();
    return 0;
}
