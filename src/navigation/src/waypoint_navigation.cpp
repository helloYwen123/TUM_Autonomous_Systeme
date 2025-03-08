#include <waypoint_navigation.h>
#include <vector>

BasicPlanner::BasicPlanner(ros::NodeHandle &nh) : nh_(nh),
                                                  max_v_(30.0),
                                                  max_a_(10.0),
                                                  current_velocity_(Eigen::Vector4d::Zero()),
                                                  current_pose_(Eigen::Vector4d::Zero())
{

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for Odometry
    sub_odom_ = nh.subscribe("current_state_est", 1, &BasicPlanner::uavOdomCallback, this);
}

void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{

    Eigen::Quaterniond qd = Eigen::Quaterniond(odom->pose.pose.orientation.x,
                                               odom->pose.pose.orientation.y,
                                               odom->pose.pose.orientation.z,
                                               odom->pose.pose.orientation.w);
    // Position
    current_pose_ << odom->pose.pose.position.x,
        odom->pose.pose.position.y,
        odom->pose.pose.position.z,
        mav_msgs::yawFromQuaternion(qd);

    // Velocity
    current_velocity_ << odom->twist.twist.linear.x,
        odom->twist.twist.linear.y,
        odom->twist.twist.linear.z,
        0.0;
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v)
{
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(const Eigen::VectorXd &goal_pos,
                                  const Eigen::VectorXd &goal_vel,
                                  mav_trajectory_generation::Trajectory *trajectory)
{

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 4;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
        mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
    start.makeStartOrEnd(current_pose_,
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);

    // add waypoint to list
    vertices.push_back(start);

    /******* Configure end point *******/
    // set end point constraints to desired position and set all derivatives to zero
    end.makeStartOrEnd(goal_pos,
                       derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      goal_vel);

    // add waypoint to list
    vertices.push_back(end);

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory &trajectory)
{
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}
