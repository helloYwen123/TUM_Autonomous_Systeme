#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>

#define STATIC_POSE 0
#define PI M_PI
#define TFOUTPUT 1

// Helper function to build a trajectory message
trajectory_msgs::MultiDOFJointTrajectoryPoint buildTrajectoryMsg(const tf::Transform &tf_trans,
                                                                   const geometry_msgs::Twist &vel,
                                                                   const geometry_msgs::Twist &acc) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
  msg.transforms.resize(1);
  msg.velocities.resize(1);
  msg.accelerations.resize(1);
  msg.transforms[0].translation.x = tf_trans.getOrigin().x();
  msg.transforms[0].translation.y = tf_trans.getOrigin().y();
  msg.transforms[0].translation.z = tf_trans.getOrigin().z();
  msg.transforms[0].rotation.x = tf_trans.getRotation().getX();
  msg.transforms[0].rotation.y = tf_trans.getRotation().getY();
  msg.transforms[0].rotation.z = tf_trans.getRotation().getZ();
  msg.transforms[0].rotation.w = tf_trans.getRotation().getW();
  msg.velocities[0] = vel;
  msg.accelerations[0] = acc;
  return msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_publisher");
  ros::NodeHandle nh;
  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);
  ros::Rate rate(500);
  ros::Time start_time = ros::Time::now();

#if TFOUTPUT
  tf::TransformBroadcaster tf_br;
#endif

  int counter = 0;
  while (ros::ok()) {
    tf::Vector3 origin(-38, 10, 6.5);
    double t = (ros::Time::now() - start_time).toSec();

    tf::Transform desired_tf;
    desired_tf.setIdentity();

    geometry_msgs::Twist vel;
    geometry_msgs::Twist acc;

#if STATIC_POSE
    tf::Vector3 offset(0, 0, 2);
    desired_tf.setOrigin(origin + offset);
    tf::Quaternion quat;
    quat.setRPY(0, 0, PI / 4);
    desired_tf.setRotation(quat);
    counter++;
    ROS_INFO("Static pose count: %d", counter);
#else
    double radius = 3.0;
    double time_scale = 5.0;
    tf::Vector3 circ_offset(radius * sin(t / time_scale),
                              radius * cos(t / time_scale),
                              2.0);
    desired_tf.setOrigin(origin + circ_offset);
    tf::Quaternion quat;
    quat.setRPY(0, 0, -t / time_scale);
    desired_tf.setRotation(quat);

    vel.linear.x = radius * cos(t / time_scale) / time_scale;
    vel.linear.y = -radius * sin(t / time_scale) / time_scale;
    vel.linear.z = 0.0;
    vel.angular.z = -1.0 / time_scale;

    acc.linear.x = -radius * sin(t / time_scale) / (time_scale * time_scale);
    acc.linear.y = -radius * cos(t / time_scale) / (time_scale * time_scale);
    acc.linear.z = 0.0;
#endif

    trajectory_msgs::MultiDOFJointTrajectoryPoint traj_msg = buildTrajectoryMsg(desired_tf, vel, acc);
    traj_pub.publish(traj_msg);

    std::stringstream ss;
    ss << "Trajectory Position: x=" << desired_tf.getOrigin().x()
       << ", y=" << desired_tf.getOrigin().y()
       << ", z=" << desired_tf.getOrigin().z();
    ROS_INFO("%s", ss.str().c_str());

#if TFOUTPUT
    tf_br.sendTransform(tf::StampedTransform(desired_tf, ros::Time::now(), "world", "av-desired"));
#endif

    ros::spinOnce();
    rate.sleep();
    counter++;
  }
  return 0;
}
