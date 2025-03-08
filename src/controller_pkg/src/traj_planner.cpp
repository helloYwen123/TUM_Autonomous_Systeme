#include "traj_planner.h"
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <sstream>
#include <cmath>

TrajectoryPlanner::TrajectoryPlanner() : n_("~") {
  // Initialize PID gains with default values
  kp << 1.2, 1.2, 0.5;
  ki << 0, 0, 0;
  kd << 0, 0, 0;
  setParam();
  ROS_INFO("TrajectoryPlanner initialized.");

  des_pos << 0.0, 0.0, 0.0;
  cur_pos_initialized_ = false;
  des_pos_initialized_ = false;

  // Subscribe to state and height topics
  current_position = nh_.subscribe("/current_state_est", 100, &TrajectoryPlanner::currentStateCallBack, this);
  expected_height_subscriber = nh_.subscribe("state_machine/expected_height", 100, &TrajectoryPlanner::height_callback, this);

  prev_time_ = ros::Time::now();
}

void TrajectoryPlanner::currentStateCallBack(const nav_msgs::Odometry& cur_state) {
  double pos_x = cur_state.pose.pose.position.x;
  double pos_y = cur_state.pose.pose.position.y;
  double pos_z = cur_state.pose.pose.position.z;
  if (std::isnan(pos_x) || std::isnan(pos_y) || std::isnan(pos_z))
    return;

  cur_pos << pos_x, pos_y, pos_z;
  cur_pos_initialized_ = true;

  ros::Time curr_time = cur_state.header.stamp;
  if (curr_time.isZero()) curr_time = ros::Time::now();
  double dt = (curr_time - prev_time_).toSec();
  if (dt <= 0) dt = 0.01;
  prev_time_ = curr_time;

  // Compute position error and filtered derivative
  Eigen::Vector3d pos_error = des_pos - cur_pos;
  ROS_INFO("Current z: %.3f, error z: %.3f", cur_pos[2], pos_error[2]);

  Eigen::Vector3d measured_dot;
  measured_dot << cur_state.twist.twist.linear.x,
                  cur_state.twist.twist.linear.y,
                  cur_state.twist.twist.linear.z;
  filtered_e_dot = 0.8 * filtered_e_dot + 0.2 * measured_dot;

  // PID output for position
  Eigen::Vector3d pos_output = kp.array() * pos_error.array() +
                               ki.array() * integrated_e.array() +
                               kd.array() * filtered_e_dot.array();

  // Update integration term with anti-windup
  for (int i = 0; i < 3; ++i) {
    if (pos_output[i] >= min_bound[i] && pos_output[i] <= max_bound[i])
      integrated_e[i] += pos_error[i] * dt;
    else if (pos_output[i] < min_bound[i])
      pos_output[i] = min_bound[i];
    else if (pos_output[i] > max_bound[i])
      pos_output[i] = max_bound[i];
  }

  // Extract yaw from quaternion using atan2
  double qw = cur_state.pose.pose.orientation.w;
  double qz = cur_state.pose.pose.orientation.z;
  double qy = cur_state.pose.pose.orientation.y;
  double qx = cur_state.pose.pose.orientation.x;
  cur_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

  // Compute yaw error and filtered derivative
  double yaw_error = des_yaw - cur_yaw;
  double measured_yaw_rate = cur_state.twist.twist.angular.z;
  filtered_e_dot_yaw = 0.8 * filtered_e_dot_yaw + 0.2 * measured_yaw_rate;

  double yaw_output = kp_yaw * yaw_error + ki_yaw * integrated_e_yaw + kd_yaw * filtered_e_dot_yaw;
  if (yaw_output >= min_bound_yaw && yaw_output <= max_bound_yaw)
    integrated_e_yaw += yaw_error * dt;
  else if (yaw_output < min_bound_yaw)
    yaw_output = min_bound_yaw;
  else if (yaw_output > max_bound_yaw)
    yaw_output = max_bound_yaw;

  // Assign computed outputs to desired velocity command
  des_vel.linear.x = pos_output.x();
  des_vel.linear.y = pos_output.y();
  des_vel.linear.z = pos_output.z();
  des_vel.angular.z = yaw_output;
}

void TrajectoryPlanner::setParam() {
  std::vector<double> p, i, d;
  if (n_.getParam("kp", p) && p.size() >= 4) {
    kp << p[0], p[1], p[2];
    kp_yaw = p[3];
    ROS_INFO("kp set to: %.2f, %.2f, %.2f, %.2f", kp[0], kp[1], kp[2], kp_yaw);
  }
  if (n_.getParam("ki", i) && i.size() >= 4) {
    ki << i[0], i[1], i[2];
    ki_yaw = i[3];
    ROS_INFO("ki set to: %.2f, %.2f, %.2f, %.2f", ki[0], ki[1], ki[2], ki_yaw);
  }
  if (n_.getParam("kd", d) && d.size() >= 4) {
    kd << d[0], d[1], d[2];
    kd_yaw = d[3];
    ROS_INFO("kd set to: %.2f, %.2f, %.2f, %.2f", kd[0], kd[1], kd[2], kd_yaw);
  }
  if (n_.getParam("z_offset", z_offset))
    ROS_INFO("z_offset set to: %.2f", z_offset);
}

void TrajectoryPlanner::pathCallBack(const nav_msgs::Path& path) {
  if (path.poses.empty()) return;
  start_explore = true;
  auto last_pose = path.poses.back();
  des_pos << last_pose.pose.position.x, last_pose.pose.position.y, expected_height;
  des_pos_initialized_ = true;

  // Calculate desired yaw using the first and last poses
  auto start_pose = path.poses.front();
  double dx = last_pose.pose.position.x - start_pose.pose.position.x;
  double dy = last_pose.pose.position.y - start_pose.pose.position.y;
  des_yaw = std::atan2(dy, dx);
  if (std::isnan(des_yaw)) des_yaw = cur_yaw;
  ROS_INFO("Desired yaw: %.2f degrees", des_yaw * 180.0 / M_PI);
}

void TrajectoryPlanner::height_callback(const std_msgs::Float64& ehmsg) {
  expected_height = ehmsg.data;
}
