// controller_node.cpp
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>

class controllerNode {
  ros::NodeHandle nh;

  // ROS callback handlers
  ros::Subscriber desired_state, current_state;
  ros::Publisher prop_speeds;
  ros::Timer timer;

  double kr, komega; 

  double m;
  double g;
  double d;
  double cf, cd;
  Eigen::Matrix3d J;
  Eigen::Vector3d e3;
  Eigen::MatrixXd F2W;

  Eigen::Vector3d x;
  Eigen::Vector3d v;
  Eigen::Matrix3d R;
  Eigen::Vector3d omega;

  Eigen::Vector3d xd;
  Eigen::Vector3d vd;
  Eigen::Vector3d ad;
  double yawd;

  double hz;

  Eigen::Matrix<double, 3, 6> K_lqr;

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in) {
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val) {
    return val > 0 ? sqrt(val) : -sqrt(-val);
  }

public:
  controllerNode() : e3(0, 0, 1), F2W(4, 4), hz(1000.0) {
      desired_state = nh.subscribe("command/trajectory", 1, &controllerNode::onDesiredState, this);
      current_state = nh.subscribe("current_state_est", 1, &controllerNode::onCurrentState, this);
      prop_speeds = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
      timer = nh.createTimer(ros::Rate(hz), &controllerNode::controlLoop, this);

      kr = 8.8;
      komega = 1.15;

      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;

      // K = [7.0711   0       0    4.9135   0       0;
      //      0      7.0711   0    0      4.9135   0;
      //      0       0     7.0711 0       0      4.9135]
      K_lqr << 7.0711, 0.0,    0.0,    4.9135, 0.0,    0.0,
               0.0,    7.0711, 0.0,    0.0,    4.9135, 0.0,
               0.0,    0.0,    7.0711, 0.0,    0.0,    4.9135;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectory& des_state) {
      geometry_msgs::Vector3 t = des_state.points[0].transforms[0].translation;
      xd << t.x, t.y, t.z;
      geometry_msgs::Vector3 v_msg = des_state.points[0].velocities[0].linear;
      vd << v_msg.x, v_msg.y, v_msg.z;
      geometry_msgs::Vector3 a_msg = des_state.points[0].accelerations[0].linear;
      ad << a_msg.x, a_msg.y, a_msg.z;
      
      tf::Quaternion q;
      tf::quaternionMsgToTF(des_state.points[0].transforms[0].rotation, q);
      yawd = tf::getYaw(q);
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state) {
      x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(cur_state.pose.pose.orientation, q);
      R = q.toRotationMatrix();
      omega << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;
      omega = R.transpose() * omega;
  }

  void controlLoop(const ros::TimerEvent& t) {
      Eigen::Vector3d ex = x - xd;
      Eigen::Vector3d ev = v - vd;
      Eigen::Matrix<double, 6, 1> x_err;
      x_err << ex, ev;

      Eigen::Vector3d u_corr = -K_lqr * x_err;

      Eigen::Vector3d F_des = m * (g * e3 + ad) + u_corr;

      Eigen::Vector3d b_3d = F_des;
      b_3d.normalize();

      Eigen::Vector3d b_1d(cos(yawd), sin(yawd), 0);
      Eigen::Vector3d b_2d = b_3d.cross(b_1d);
      b_2d.normalize();
      Eigen::Vector3d b_1d_true = b_2d.cross(b_3d);
      b_1d_true.normalize();

      Eigen::Matrix3d Rd;
      Rd << b_1d_true, b_2d, b_3d;

      Eigen::Vector3d er = 0.5 * Vee(Rd.transpose() * R - R.transpose() * Rd);
      Eigen::Vector3d eomega = -omega;

      double f = F_des.dot(R * e3);

      Eigen::Vector3d torques = -kr * er - komega * eomega + omega.cross(J * omega);

      Eigen::Vector4d wrench(f, torques.x(), torques.y(), torques.z());

      double d_hat = d / sqrt(2);
      Eigen::Matrix4d F;
      F << cf,       cf,       cf,       cf,
           cf*d_hat, cf*d_hat, -cf*d_hat, -cf*d_hat,
          -cf*d_hat, cf*d_hat,  cf*d_hat, -cf*d_hat,
           cd,      -cd,       cd,      -cd;
      Eigen::Vector4d props = F.inverse() * wrench;

      mav_msgs::Actuators msg;
      msg.angular_velocities.resize(4);
      msg.angular_velocities[0] = signed_sqrt(props[0]);
      msg.angular_velocities[1] = signed_sqrt(props[1]);
      msg.angular_velocities[2] = signed_sqrt(props[2]);
      msg.angular_velocities[3] = signed_sqrt(props[3]);
      prop_speeds.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller_node");
  ROS_INFO_NAMED("controller", "Controller started!");
  controllerNode node;
  ros::spin();
  return 0;
}
