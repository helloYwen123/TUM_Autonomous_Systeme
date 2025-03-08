#ifndef TRAJ_PLANNER_H
#define TRAJ_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>

class TrajectoryPlanner {
public:
  // Subscribers
  ros::Subscriber current_position;
  ros::Subscriber planned_path;
  ros::Subscriber expected_height_subscriber;

  // PID gains for position control (3 components) and yaw control
  Eigen::Vector3d kp, ki, kd;
  double kp_yaw, ki_yaw, kd_yaw;

  // Current and desired states in the world frame
  Eigen::Vector3d cur_pos;
  Eigen::Vector3d des_pos;
  double cur_yaw;
  double des_yaw;
  double expected_height{4.0};
  double z_offset{0.0};

  // Flags and desired command outputs
  bool cur_pos_initialized_;
  bool des_pos_initialized_;
  bool start_explore{false};
  geometry_msgs::Twist des_vel;
  geometry_msgs::Twist des_acc;

  TrajectoryPlanner();

  void currentStateCallBack(const nav_msgs::Odometry& cur_state);
  void setParam();
  void pathCallBack(const nav_msgs::Path& path);
  void height_callback(const std_msgs::Float64& ehmsg);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle n_;

  // PID integration and derivative filtering
  Eigen::Vector3d integrated_e{0.0, 0.0, 0.0};
  Eigen::Vector3d filtered_e_dot{0.0, 0.0, 0.0};
  double integrated_e_yaw{0.0};
  double filtered_e_dot_yaw{0.0};

  // Output saturation bounds
  Eigen::Vector3d min_bound{-0.02, -0.02, -0.02};
  Eigen::Vector3d max_bound{0.02, 0.02, 0.02};
  double min_bound_yaw{-0.2};
  double max_bound_yaw{0.2};

  // For dt computation in the PID update
  ros::Time prev_time_;
};

#endif // TRAJ_PLANNER_H
