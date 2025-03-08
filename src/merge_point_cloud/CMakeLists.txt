cmake_minimum_required(VERSION 3.0.2)
project(merge_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
  message_filters
)

find_package(PCL REQUIRED)

catkin_package(
 LIBRARIES merge_point_cloud
 CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions std_srvs message_filters
 DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(merge_point_cloud_node src/merge_point_cloud_node.cpp)

target_link_libraries(merge_point_cloud_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
