cmake_minimum_required(VERSION 3.0.2)
project(state_machine_pkg)


add_compile_options(-std=c++14)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  mav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  tf2_geometry_msgs    
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

################################################
## catkin specific configuration
################################################
catkin_package(
  CATKIN_DEPENDS eigen_conversions geometry_msgs mav_msgs roscpp tf tf_conversions tf2_geometry_msgs std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(state_machine_pkg
  src/state_machine.cpp
  src/cubic_spline.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(state_machine_pkg ${catkin_LIBRARIES})

