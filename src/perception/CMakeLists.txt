cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(PCL REQUIRED) 

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  std_msgs
  tf
  image_geometry
  pcl_conversions
  pcl_ros
  depth_image_proc
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS 
  cv_bridge
  std_msgs
  tf
  image_geometry
  pcl_conversions
  pcl_ros
  depth_image_proc
  sensor_msgs
  geometry_msgs
)

add_executable(light_detection_node src/light_detection.cpp)
add_dependencies(light_detection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(light_detection_node ${catkin_LIBRARIES})


include_directories(
  ${catkin_INCLUDE_DIRS}
)

