#include "Optics.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <vector>
#include <mutex>

// Frontier exploration class that processes octomap updates and detects frontiers.
class Frontier {
public:
  Frontier();

private:
  ros::NodeHandle node_handle;
  ros::Subscriber state_subscriber;
  ros::Subscriber map_subscriber;
  ros::Publisher goal_publisher;
  ros::Timer exploration_timer;

  std::mutex frontier_mutex;

  // Instead of storing an AbstractOcTree, we store two pointers:
  // one for standard OcTree and one for ColorOcTree.
  std::shared_ptr<octomap::OcTree> oc_tree_ptr;
  std::shared_ptr<octomap::ColorOcTree> color_tree_ptr;

  // Current robot position from odometry.
  octomap::point3d current_position;
  std::vector<geometry_msgs::Point> frontier_points;
  int octomap_resolution;
  int max_distance; 
  std::vector<pcl::PointIndicesPtr> cluster_indices;

  // Callback functions and processing methods.
  void currentStateCallback(const nav_msgs::Odometry &msg);
  void mapUpdateCallback(const octomap_msgs::OctomapConstPtr &msg);
  void explorationTimerCallback(const ros::TimerEvent &event);

  pcl::PointCloud<pcl::PointXYZ>::Ptr generateFrontierCloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr identifyLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointXYZ calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud);

  void detectFrontiers();
  bool isFrontierPoint(const octomap::point3d &coord);
  void publishGoal(const pcl::PointXYZ &goal);
};

Frontier::Frontier() {
  // Read parameters (with default values if not set)
  node_handle.param<int>("/octomap_server/resolution", octomap_resolution, 2);
  node_handle.param<int>("max_distance", max_distance, 35);
  node_handle.getParam("/octomap_server/resolution", octomap_resolution);

  // Subscribers and publishers
  state_subscriber = node_handle.subscribe("current_state_est", 1, &Frontier::currentStateCallback, this);
  map_subscriber   = node_handle.subscribe("octomap_full", 1, &Frontier::mapUpdateCallback, this);
  goal_publisher   = node_handle.advertise<geometry_msgs::Point>("frontier_goal", 1);

  ROS_INFO_STREAM("Frontier constructor: octomap_resolution=" << octomap_resolution);
  ROS_INFO_STREAM("Frontier constructor: max_distance=" << max_distance 
                  << " (might be uninitialized if not set anywhere!)" );

  // Create a timer (0.3 Hz) to periodically run the exploration routine.
  exploration_timer = node_handle.createTimer(
      ros::Duration(1 / 0.3),
      &Frontier::explorationTimerCallback, 
      this
  );
}

void Frontier::currentStateCallback(const nav_msgs::Odometry &msg) {
  // Update current position from odometry.
  current_position = octomap::point3d(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z
  );
}

void Frontier::mapUpdateCallback(const octomap_msgs::OctomapConstPtr &msg) {
  std::lock_guard<std::mutex> lock(frontier_mutex);

  // Convert the incoming ROS Octomap message to an AbstractOcTree.
  std::unique_ptr<octomap::AbstractOcTree> tree{octomap_msgs::msgToMap(*msg)};
  if (!tree) {
    ROS_ERROR("mapUpdateCallback: msgToMap returned null!");
    return;
  }
  std::string treeType = tree->getTreeType();
  
  // Check the tree type and store in the appropriate pointer.
  if (treeType == "ColorOcTree") {
    auto ctree = dynamic_cast<octomap::ColorOcTree*>(tree.get());
    if (!ctree) {
      ROS_ERROR("mapUpdateCallback: dynamic_cast<ColorOcTree*> failed!");
      return;
    }
    // Save a copy of the ColorOcTree.
    color_tree_ptr = std::make_shared<octomap::ColorOcTree>(*ctree);
    // Reset the standard OcTree pointer.
    oc_tree_ptr.reset();
    ROS_INFO("mapUpdateCallback: Received ColorOcTree, stored as color_tree_ptr.");
  }
  else if (treeType == "OcTree") {
    auto otree = dynamic_cast<octomap::OcTree*>(tree.get());
    if (!otree) {
      ROS_ERROR("mapUpdateCallback: dynamic_cast<OcTree*> failed!");
      return;
    }
    oc_tree_ptr = std::make_shared<octomap::OcTree>(*otree);
    color_tree_ptr.reset();
    ROS_INFO("mapUpdateCallback: Received OcTree, stored as oc_tree_ptr.");
  }
  else {
    ROS_WARN_STREAM("mapUpdateCallback: Unknown tree type: " << treeType << ". Ignoring update.");
  }
}

void Frontier::explorationTimerCallback(const ros::TimerEvent &event) {
  {
    std::lock_guard<std::mutex> lock(frontier_mutex);
    if (!oc_tree_ptr && !color_tree_ptr) {
      ROS_ERROR_STREAM("explorationTimerCallback: No octree available yet!");
      return;
    }
  }

  // Run frontier detection.
  detectFrontiers();

  // Generate a point cloud from frontier points.
  auto frontier_cloud = generateFrontierCloud();

  // Perform OPTICS clustering on the frontier point cloud.
  Optics::optics<pcl::PointXYZ>(frontier_cloud, 5, 10.0, cluster_indices);

  // Identify the largest cluster that meets the condition.
  auto largest_cluster_cloud = identifyLargestCluster(frontier_cloud);

  // Compute the centroid (goal) of the largest cluster.
  auto goal = calculateGoal(*largest_cluster_cloud);
  ROS_INFO("Frontier goal point set to: (%f, %f, %f)", goal.x, goal.y, goal.z);

  // Publish the goal if it meets the required condition.
  publishGoal(goal);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Frontier::generateFrontierCloud() {
  // Create a new point cloud for frontier points.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->header.frame_id = "world";
  cloud->is_dense = false;
  for (const auto &point : frontier_points) {
    cloud->points.emplace_back(point.x, point.y, point.z);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Frontier::identifyLargestCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::vector<std::pair<int, pcl::PointIndices::Ptr>> sorted_clusters;
  sorted_clusters.reserve(cluster_indices.size());
  for (auto &c : cluster_indices) {
    sorted_clusters.emplace_back(c->indices.size(), c);
  }
  std::sort(sorted_clusters.begin(), sorted_clusters.end(),
            [](auto &a, auto &b) {
              return a.first > b.first;
            });

  // Return the first cluster whose centroid meets the condition (x <= -340.0).
  for (auto &sc : sorted_clusters) {
    auto indices_ptr = sc.second;
    auto candidate_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int idx : indices_ptr->indices) {
      candidate_cloud->points.push_back(cloud->points[idx]);
    }
    auto centroid = calculateGoal(*candidate_cloud);
    if (centroid.x <= -340.0) {
      return candidate_cloud;
    }
  }
  // Return an empty cloud if no cluster meets the condition.
  auto empty_cluster = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  return empty_cluster;
}

pcl::PointXYZ Frontier::calculateGoal(pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::PointXYZ goal(0, 0, 0);
  if (cloud.points.empty()) return goal;
  for (const auto &point : cloud.points) {
    goal.x += point.x;
    goal.y += point.y;
    goal.z += point.z;
  }
  goal.x /= cloud.points.size();
  goal.y /= cloud.points.size();
  goal.z /= cloud.points.size();
  return goal;
}

void Frontier::detectFrontiers() {
  std::lock_guard<std::mutex> lock(frontier_mutex);
  frontier_points.clear();

  ROS_INFO_STREAM("detectFrontiers: max_distance=" << max_distance
                  << ", current_position=(" << current_position.x() 
                  << ", " << current_position.y() 
                  << ", " << current_position.z() << ")");
  ROS_INFO_STREAM("Detecting frontiers: octomap_resolution=" << octomap_resolution
                  << ", max_distance=" << max_distance);

  // Define a bounding box around the current position.
  octomap::point3d minPt(
      current_position.x() - max_distance,
      current_position.y() - max_distance,
      current_position.z() - max_distance
  );
  // Set the x-maximum based on a specific condition.
  float x_max = std::min(current_position.x() + max_distance, -340.0f);
  octomap::point3d maxPt(
      x_max,
      current_position.y() + max_distance,
      current_position.z() + max_distance
  );

  ROS_INFO_STREAM("  bounding box: minPt=(" 
                  << minPt.x() << ", " 
                  << minPt.y() << ", " 
                  << minPt.z() 
                  << ") , maxPt=(" 
                  << maxPt.x() << ", " 
                  << maxPt.y() << ", " 
                  << maxPt.z() << ")");

  // Determine which tree pointer is active and iterate over its leaves.
  if (color_tree_ptr) {
    // Use the ColorOcTree pointer.
    for (auto it = color_tree_ptr->begin_leafs_bbx(minPt, maxPt), end = color_tree_ptr->end_leafs_bbx(); it != end; ++it) {
      octomap::point3d coord = it.getCoordinate();
      if (!color_tree_ptr->isNodeOccupied(*it)) {
        if (isFrontierPoint(coord)) {
          geometry_msgs::Point fp;
          fp.x = coord.x();
          fp.y = coord.y();
          fp.z = coord.z();
          frontier_points.push_back(fp);
        }
      }
    }
  } else if (oc_tree_ptr) {
    // Use the standard OcTree pointer.
    for (auto it = oc_tree_ptr->begin_leafs_bbx(minPt, maxPt), end = oc_tree_ptr->end_leafs_bbx(); it != end; ++it) {
      octomap::point3d coord = it.getCoordinate();
      if (!oc_tree_ptr->isNodeOccupied(*it)) {
        if (isFrontierPoint(coord)) {
          geometry_msgs::Point fp;
          fp.x = coord.x();
          fp.y = coord.y();
          fp.z = coord.z();
          frontier_points.push_back(fp);
        }
      }
    }
  } else {
    ROS_ERROR("detectFrontiers: No octree available!");
  }
}

bool Frontier::isFrontierPoint(const octomap::point3d &coord) {
  // Define neighbor offsets.
  std::vector<octomap::point3d> offsets = {
    {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0},
    {0,0,1}, {0,0,-1}, {1,1,0}, {-1,-1,0},
    {1,-1,0}, {-1,1,0}, {1,0,1}, {-1,0,-1},
    {0,1,1}, {0,-1,-1}, {-1,0,1}, {1,0,-1},
    {0,-1,1}, {0,1,-1}, {1,1,1}, {-1,-1,-1},
    {1,-1,1}, {-1,1,-1}, {1,1,-1}, {-1,-1,1},
    {1,-1,-1}, {-1,1,1}
  };

  // Check each neighbor; if any neighbor is unknown, the point is considered a frontier.
  if (color_tree_ptr) {
    for (auto &offset : offsets) {
      octomap::point3d neighbor = coord + offset * octomap_resolution;
      auto node = color_tree_ptr->search(neighbor);
      if (!node) {
        return true;
      }
    }
  } else if (oc_tree_ptr) {
    for (auto &offset : offsets) {
      octomap::point3d neighbor = coord + offset * octomap_resolution;
      auto node = oc_tree_ptr->search(neighbor);
      if (!node) {
        return true;
      }
    }
  }
  return false;
}

void Frontier::publishGoal(const pcl::PointXYZ &goal) {
  if (goal.x <= -340.0) {
    geometry_msgs::Point goal_msg;
    goal_msg.x = goal.x;
    goal_msg.y = goal.y;
    goal_msg.z = goal.z;
    goal_publisher.publish(goal_msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "frontier_exploration");
  Frontier frontier_exploration;
  ros::spin();
  return 0;
}
