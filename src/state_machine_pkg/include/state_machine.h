#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>


class StateBase;


enum class State
{
    takeoff,
    to_cave,
    hover,
    landing,
    turn,
    forward
};

class StateMachine
{
public:
    StateMachine();

    
    geometry_msgs::Point getNextGoalPoint();
    geometry_msgs::PoseStamped pointToPoseStamped(const geometry_msgs::Point &point, const std::string &frame_id, double yaw = 0.0);
    void addGoalPoint(double x, double y, double z);
    void onCurrentState(const nav_msgs::Odometry &cur_state);
    void onPlannedPath(const nav_msgs::Path::ConstPtr &msg);
    void state_machine_mission(const ros::TimerEvent &t);

    bool in_range(double low, double high, double x);
    bool goal_reached();
    void set_waypoint(tf::Vector3 pos, tf::Quaternion q,
                      tf::Vector3 lin_vel = tf::Vector3(0,0,0),
                      tf::Vector3 ang_vel = tf::Vector3(0,0,0),
                      tf::Vector3 lin_acc = tf::Vector3(0,0,0));
    void set_position();

    
    void sendGoal(const geometry_msgs::Point &point, const std::string &frame_id, double yaw = 0.0);
    void updateCurrentPosition();
    const geometry_msgs::Point& getLastGoal() const { return goalpoint; }

    
    ros::NodeHandle nh;
    ros::Publisher desired_state_pub_;
    ros::Publisher goal_position_pub_;
    ros::Subscriber current_state_sub_;
    ros::Subscriber path_sub_;
    ros::Timer state_machine_timer_;
    tf::TransformBroadcaster br;

    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d omega_;
    tf::Quaternion quat_;
    double yaw_;

    
    std::vector<geometry_msgs::Point> goalpoints;
    size_t current_goal_index = 0;
    geometry_msgs::Point goalpoint;
    std::vector<Eigen::Vector4d> goalpoints_path_vec;

  
    bool goal_sent_once = false;
    double yaw_des = 0.0;
    Eigen::Vector3d cur_position;  


    tf::Vector3 origin_;


    std::unique_ptr<StateBase> current_state;
};

#endif // STATE_MACHINE_H

