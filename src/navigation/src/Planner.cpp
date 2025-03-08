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

#include "Planner.h"

namespace {
  // Copy logOdds from each leaf in color_tree to a new OcTree, preserving probabilities
  std::shared_ptr<octomap::OcTree> convertColorToOccupancy(const octomap::ColorOcTree &color_tree) {
    auto occ_tree = std::make_shared<octomap::OcTree>(color_tree.getResolution());
    for (auto it = color_tree.begin_leafs(), end = color_tree.end_leafs(); it != end; ++it) {
      float log_odds = it->getLogOdds();
      auto node = occ_tree->updateNode(it.getKey(), false); 
      node->setLogOdds(log_odds);
    }
    occ_tree->updateInnerOccupancy();
    return occ_tree;
  }
}

Planner::Planner()
    : space(ob::StateSpacePtr(new ob::RealVectorStateSpace(3))),
      goal(ob::ScopedState<ob::RealVectorStateSpace>(space)),
      start(ob::ScopedState<ob::RealVectorStateSpace>(space)),
      plan_flag(false),
      bounds(3)
{
    // Set up subscriptions and publisher
    current_state_sub = nh.subscribe("/current_state_est", 1, &Planner::updateOdom, this);
    octomap_sub = nh.subscribe("octomap_full", 1, &Planner::updateMap, this);
    frontier_goal_sub = nh.subscribe("frontier_goal", 1, &Planner::setGoal, this);
    planned_path_pub = nh.advertise<nav_msgs::Path>("planned_path", 0);

    // Define collision object
    aircraftObject = std::make_shared<fcl::CollisionObject<double>>(
        std::make_shared<fcl::Box<double>>(2, 2, 2));

    // Set initial bounds
    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    bounds.setLow(2, 0.5);
    bounds.setHigh(2, 3.5);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Configure OMPL
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
    si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));
    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

    nh.param<std::string>("planner_method", planner_method_, "prmstar");
    ROS_INFO("Planner constructor: selected method = %s", planner_method_.c_str());
    ROS_INFO("Planner Initialized (RRTstar / InformedRRTstar / PRMstar).");
}

Planner::~Planner()
{
}

void Planner::setGoal(const geometry_msgs::PointConstPtr &msg)
{
    // Set goal from incoming point and plan if valid
    std::lock_guard<std::mutex> lock(mutex_flag);

    goal[0] = msg->x;
    goal[1] = msg->y;
    goal[2] = msg->z;

    plan_flag = true;
    pdef->clearSolutionPaths();

    ob::State *goalState = space->allocState();
    goalState->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal[0];
    goalState->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal[1];
    goalState->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal[2];

    if (isStateValid(goalState)) {
        pdef->setStartAndGoalStates(start, goal);
        pdef->setOptimizationObjective(getPathLengthObjWithCostToGo(si));

        if (planner_method_ == "rrtstar") {
            o_plan = ob::PlannerPtr(new og::RRTstar(si));
            ROS_INFO("Using RRTstar");
        }
        else if (planner_method_ == "informedrrtstar") {
            o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));
            ROS_INFO("Using InformedRRTstar");
        }
        else {
            o_plan = ob::PlannerPtr(new og::PRMstar(si));
            ROS_INFO("Using PRMstar (default)");
        }

        o_plan->setProblemDefinition(pdef);
        o_plan->setup();

        plan();
        plan_flag = false;
        ROS_INFO_STREAM("Goal set to: (" << msg->x << ", " << msg->y << ", " << msg->z << ")");
    }
    else {
        ROS_ERROR_STREAM("Invalid goal: (" << msg->x << ", " << msg->y << ", " << msg->z << ")");
    }

    space->freeState(goalState);
}

void Planner::updateMap(const octomap_msgs::OctomapConstPtr &msg)
{
    // Update internal octree from incoming map; handle both OcTree and ColorOcTree
    std::lock_guard<std::mutex> lock(mutex_flag);
    if (plan_flag) return;

    std::unique_ptr<octomap::AbstractOcTree> tree{octomap_msgs::msgToMap(*msg)};
    if (!tree) {
        ROS_ERROR("updateMap: msgToMap returned null!");
        return;
    }

    if (tree->getTreeType() == "OcTree") {
        auto oc = dynamic_cast<octomap::OcTree*>(tree.get());
        if (!oc) {
            ROS_ERROR("updateMap: dynamic_cast<OcTree*> failed!");
            return;
        }
        octree = std::make_shared<octomap::OcTree>(*oc);
        color_octree.reset();
        ROS_INFO("Planner updateMap: got normal OcTree.");
    }
    else if (tree->getTreeType() == "ColorOcTree") {
        auto color_ptr = dynamic_cast<octomap::ColorOcTree*>(tree.get());
        if (!color_ptr) {
            ROS_ERROR("updateMap: dynamic_cast<ColorOcTree*> failed!");
            return;
        }
        color_octree = std::make_shared<octomap::ColorOcTree>(*color_ptr);
        ROS_INFO("Planner updateMap: got ColorOcTree.");

        // Convert to OcTree with original logOdds
        octree = convertColorToOccupancy(*color_ptr);
    }
    else {
        ROS_ERROR_STREAM("updateMap: Unknown tree type: " << tree->getTreeType());
        return;
    }

    if (!octree) {
        ROS_ERROR("Planner updateMap: The occupancy octree is null!");
        return;
    }

    double minBounds[3], maxBounds[3];
    octree->getMetricMin(minBounds[0], minBounds[1], minBounds[2]);
    octree->getMetricMax(maxBounds[0], maxBounds[1], maxBounds[2]);

    // Create FCL collision object
    auto fclTree = std::make_shared<fcl::OcTree<double>>(octree);
    auto collisionGeometry = std::static_pointer_cast<fcl::CollisionGeometry<double>>(fclTree);
    treeObj = std::make_shared<fcl::CollisionObject<double>>(collisionGeometry);

    // Update OMPL bounds
    bounds.setLow(0, minBounds[0]);
    bounds.setHigh(0, maxBounds[0]);
    bounds.setLow(1, minBounds[1]);
    bounds.setHigh(1, maxBounds[1]);
    bounds.setLow(2, minBounds[2]);
    bounds.setHigh(2, maxBounds[2]);
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
}

void Planner::updateOdom(const nav_msgs::Odometry &msg)
{
    // Update start state from odometry
    start[0] = msg.pose.pose.position.x;
    start[1] = msg.pose.pose.position.y;
    start[2] = msg.pose.pose.position.z;
}

void Planner::plan()
{
    // Attempt to solve within 5 seconds, publish path if successful
    ob::PlannerStatus solved = o_plan->solve(5);
    if (solved) {
        ROS_INFO("Planner found a solution.");
        auto path = pdef->getSolutionPath();
        auto pathGeo = path->as<og::PathGeometric>();
        pathGeo->printAsMatrix(std::cout);

        planned_path_pub.publish(pathMsg(pathGeo));
        o_plan->clear();

        // Optional path smoothing
        og::PathSimplifier pathBSpline(si);
        path_smooth = new og::PathGeometric(*pathGeo);
        pathBSpline.smoothBSpline(*path_smooth);
    } else {
        ROS_ERROR("No solution found within time limit.");
    }
}

bool Planner::isStateValid(const ob::State *state)
{
    // Collision check with FCL
    const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();

    fcl::Vector3<double> translation(pos->values[0],
                                     pos->values[1],
                                     pos->values[2]);
    aircraftObject->setTranslation(translation);

    fcl::CollisionRequest<double> request;
    fcl::CollisionResult<double> result;

    if (treeObj && aircraftObject) {
        fcl::collide(aircraftObject.get(), treeObj.get(), request, result);
        return !result.isCollision();
    }
    return false;
}

ob::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr &si)
{
    // Use path length as optimization objective
    return std::make_shared<ob::PathLengthOptimizationObjective>(si);
}

nav_msgs::Path Planner::pathMsg(og::PathGeometric *pathGeo)
{
    // Convert path to ROS Path message
    std::vector<Eigen::Vector3d> pathPoints;
    const auto &states = pathGeo->getStates();

    for (size_t i = 0; i < states.size(); ++i) {
        const auto *st = states[i]->as<ob::RealVectorStateSpace::StateType>();
        pathPoints.emplace_back(st->values[0], st->values[1], st->values[2]);
    }

    return vectorMsg(pathPoints);
}

nav_msgs::Path Planner::vectorMsg(const std::vector<Eigen::Vector3d> &pathVector)
{
    // Build nav_msgs::Path from a list of 3D points
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.poses.resize(pathVector.size());

    for (int i = 0; i < (int)pathVector.size(); i++) {
        msg.poses[i].pose.position.x = pathVector[i][0];
        msg.poses[i].pose.position.y = pathVector[i][1];
        msg.poses[i].pose.position.z = pathVector[i][2];
    }
    return msg;
}

std::vector<std::tuple<double, double, double>> Planner::getSmoothPath()
{
    // Return smoothed path as a vector of (x, y, z)
    std::vector<std::tuple<double, double, double>> path;
    if (!path_smooth) return path;

    for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++) {
        const auto *pos = path_smooth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();
        path.emplace_back(pos->values[0], pos->values[1], pos->values[2]);
    }
    return path;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Planner");
    Planner n;
    ros::spin();
    return 0;
}