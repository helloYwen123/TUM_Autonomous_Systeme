#include "ros/ros.h"
#include <waypoint_navigation.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h> // For converting quaternion to yaw
#include <iostream>
#include <Eigen/Dense>

// Global pointer to BasicPlanner
BasicPlanner* planner = nullptr;
Eigen::Vector4d goal_velocity; // Extended to include yaw velocity
Eigen::Vector4d last_goal_pose = Eigen::Vector4d::Zero(); // Includes position and yaw

// Function to check if two poses are different (now considering yaw)
bool isDifferentPose(const Eigen::Vector4d& pose1, const Eigen::Vector4d& pose2) {
    const double threshold = 0.01; // Define a threshold for position differences
    // Only compare the first three elements (X, Y, Z) for position difference
    return (pose1.head<3>() - pose2.head<3>()).norm() > threshold;
}

// Callback function for goal pose updates (including yaw)
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double yaw = tf::getYaw(msg->pose.orientation); // Convert quaternion to yaw
    Eigen::Vector4d goal_pose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, yaw);

    if (isDifferentPose(goal_pose, last_goal_pose)) {
        ROS_INFO_STREAM("Received new goal pose: " << goal_pose.transpose());

        mav_trajectory_generation::Trajectory trajectory;
        // Ensure planTrajectory can handle the extended goal_pose which includes yaw
        planner->planTrajectory(goal_pose, goal_velocity, &trajectory); 
        planner->publishTrajectory(trajectory);

        last_goal_pose = goal_pose; // Update the last goal pose
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_planner");  // Initialize ROS
    ros::NodeHandle n;  // Create a NodeHandle after ROS has been initialized

    planner = new BasicPlanner(n);
    goal_velocity << 0.0, 0.0, 0.0, 0.0; // Extended to include yaw velocity

    ros::Subscriber goal_sub = n.subscribe("goal_position", 1, goalCallback);

    ros::spin();  // Keep the node running and listening to callbacks

    delete planner;
    return 0;
}

