/*
 *   OctomapPlanner
 *
 *   Copyright (C) 2018  ArduPilot
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   Author Ayush Gaud <ayush.gaud[at]gmail.com>
 */


#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// Three planner headers
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/StateSpace.h>
#include <ompl/config.h>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>
#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner
{
public:
    Planner();
    ~Planner();

    // Set the goal state using a geometry_msgs::Point
    void setGoal(const geometry_msgs::PointConstPtr &msg);

    // Attempt to solve a motion planning problem
    void plan();

    // Update internal octree from incoming Octomap data
    void updateMap(const octomap_msgs::OctomapConstPtr &msg);

    // Update the start state from odometry
    void updateOdom(const nav_msgs::Odometry &msg);

    // Convert an OMPL path to a ROS nav_msgs::Path
    nav_msgs::Path pathMsg(og::PathGeometric *pathGeo);

    // Build a nav_msgs::Path from a list of 3D points
    nav_msgs::Path vectorMsg(const std::vector<Eigen::Vector3d> &pathVector);

    // Retrieve the smoothed path as (x, y, z) tuples
    std::vector<std::tuple<double, double, double>> getSmoothPath();

    // Collision check for a given state
    bool isStateValid(const ob::State *state);

    // Define the path length optimization objective
    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si);

private:
    ros::NodeHandle nh;

    ros::Subscriber current_state_sub, octomap_sub, frontier_goal_sub;
    ros::Publisher planned_path_pub;

    // OMPL core data structures
    ob::StateSpacePtr space;
    ob::ScopedState<ob::RealVectorStateSpace> start;
    ob::ScopedState<ob::RealVectorStateSpace> goal;
    ob::SpaceInformationPtr si;
    ob::ProblemDefinitionPtr pdef;
    ob::PlannerPtr o_plan;

    ob::RealVectorBounds bounds = ob::RealVectorBounds(3);
    og::PathGeometric *path_smooth = nullptr;

    std::mutex mutex_flag;
    bool plan_flag;

    // Store OcTree and collision objects
    std::shared_ptr<octomap::OcTree> octree;
    std::shared_ptr<fcl::CollisionObject<double>> treeObj;
    std::shared_ptr<fcl::CollisionObject<double>> aircraftObject;
    std::shared_ptr<octomap::ColorOcTree> color_octree;

    // Planner method (prmstar / rrtstar / informedrrtstar)
    std::string planner_method_;
};

#endif