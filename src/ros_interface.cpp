/**
 * MIT License
 *
 * Copyright (c) 2019 Sean Crutchlow
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ros_interface.cpp
 * @brief Source file for ROS interface.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include "theta_star/ros_interface.h"

namespace theta_star {
  RosInterface::RosInterface(const ros::NodeHandle& _p_nh) {
    /// Node Handle Assignment
    p_nh_ = _p_nh;

    /// Params
    LoadParams();

    /// Initialize Class Variables
    InitVariables();

    /// Initialize Publishers, Subscribers, and Services
    InitPubSubSrv();
  }

  RosInterface::~RosInterface() {
    /// Shutdown Pubs, Subs, and Srvs
    grid_sub_.shutdown();
    plan_srv_server_.shutdown();
  }

  void RosInterface::LoadParams() {
    /// Namespaces
    p_nh_.param<std::string>("node_name", node_name_,
      "theta_star");
    std::string node_ns_ = "/" + node_name_ + "/";

    /// Check Topics
    VerifyParam(node_ns_, "grid_sub_topic", grid_sub_topic_);
    VerifyParam(node_ns_, "goal_sub_topic", goal_sub_topic_);

    /// Check Frame IDs
    VerifyParam(node_ns_, "relative_frame_id", relative_frame_id_);

    /// Check Moves & Costs
    std::vector<std::string> moves_list;
    VerifyParam(node_ns_ + "moves" + "/", "list", moves_list);
    VerifyParam(node_ns_ + "moves" + "/", "costs", costs_);

    if (moves_list.empty()) {
      ROS_ERROR("[%s]: Moves list param for theta_star not populated."
        " Exiting...",
        ros::this_node::getName().c_str());
      exit(1);
    } else {
      for (auto it : moves_list) {
        std::vector<int> move;
        VerifyParam(node_ns_ + "moves" + "/", it, move);
        moves_.push_back(std::make_pair(move.at(0), move.at(1)));
      }
    }
  }

  void RosInterface::InitVariables() {
    /// Grid Information
    grid_resolution_ = 0.0;
    num_rows_ = 0;
    num_cols_ = 0;

    /// Theta Star
    ts_ptr_ = std::make_shared<ThetaStar>(moves_, costs_);
  }

  void RosInterface::InitPubSubSrv() {
    /// Subscribers
    grid_sub_ = p_nh_.subscribe<nav_msgs::OccupancyGrid>(
      grid_sub_topic_, 1, &RosInterface::OccupancyGridCB, this);

    /// Services
    plan_srv_server_ = p_nh_.advertiseService("plan",
      &RosInterface::PlanSrv, this);
  }

  ThetaStar::coord_t RosInterface::Cont2Discrete(
    const geometry_msgs::Pose& _pose) {
    // TODO(Sean):
    //  1) Possibly provide signature including TF::Transform
    //     to minimize discrepancy of origin. Retrieve world
    //     and grid frame from loaded params
    //  2) Find out if it is common to apply a rotation
    //     to the origin of the occupancy grid. If required,
    //     need a 2D point transformation.
    int x = ((grid_origin_.position.x - _pose.position.x) / grid_resolution_);
    int y = ((grid_origin_.position.y - _pose.position.y) / grid_resolution_);

    /// Offset for vector indexing
    if (x > 0)
      x -= 1;
    if (y > 0)
      y -= 1;

    return std::make_pair(x, y);
  }

  nav_msgs::Path RosInterface::Discrete2Cont(const geometry_msgs::Pose& _start,
    const std::vector<ThetaStar::coord_t>& _coords) {
    /// Path to be populated
    nav_msgs::Path path;

    /// Add intial pose to path
    geometry_msgs::PoseStamped first_pose;
    first_pose.pose = _start;
    path.poses.push_back(first_pose);

    /// Convert start orientation to Euler angle (yaw)
    tf2::Quaternion start(_start.orientation.x,
                          _start.orientation.y,
                          _start.orientation.z,
                          _start.orientation.w);

    /// Initial heading from start (+yaw CCW)
    double absolute_theta = -start.getAngle();

    for (int i = 0; i < (_coords.size() - 1); i++) {
      geometry_msgs::PoseStamped pose;

      /// Deltas between nodes
      double dx = ((_coords[i+1].first - _coords[i].first)
                    / grid_resolution_);
      double dy = ((_coords[i+1].second - _coords[i].second)
                    / grid_resolution_);

      /// Convert back to absolute coordinates
      double abs_x = grid_origin_.position.x - _coords[i+1].first
                      - grid_resolution_;
      double abs_y = grid_origin_.position.y - _coords[i+1].second
                      - grid_resolution_;

      /// atan2 for heading i, i+1
      double relative_theta = -atan2(dy, dx);
      absolute_theta = std::fmod(relative_theta + absolute_theta, 2.0 * M_PI);

      /// Euler to Quaternion
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, absolute_theta);
      q.normalize();

      /// Populate pose
      pose.pose.position.x = abs_x;
      pose.pose.position.y = abs_y;
      pose.pose.orientation = tf2::toMsg(q);

      /// Add pose to path
      path.poses.push_back(pose);
    }
    /// Add Header - Stamp
    path.header.stamp = ros::Time::now();

    return path;
  }

  bool RosInterface::PlanSrv(nav_msgs::GetPlan::Request& _req,
    nav_msgs::GetPlan::Response& _res) {
    /// Convert continuous start and goal poses discrete coordinates
    /// for path planner
    ThetaStar::coord_t start = Cont2Discrete(_req.start.pose);
    ThetaStar::coord_t goal = Cont2Discrete(_req.goal.pose);

    /// Store optimal path in std::pair form
    std::vector<ThetaStar::coord_t> path = ts_ptr_->Nodes2Pairs(
      ts_ptr_->Plan(start, goal));

    /// Path from start to goal could not be found if empty
    if (path.empty()) {
      ROS_ERROR("[%s] Empty path. Goal could not be found.",
        ros::this_node::getName().c_str());

      return false;
    }
    /// Convert to ROS path type
    _res.plan = Discrete2Cont(_req.start.pose, path);

    return true;
  }

  void RosInterface::OccupancyGridCB(
    const nav_msgs::OccupancyGrid::ConstPtr& _msg) {
    /// Grid Information
    grid_resolution_ = _msg->info.resolution;

    assert((std::fabs(grid_resolution_) > 1e-8)
      && "Grid resolution is 0 value");

    grid_origin_ = _msg->info.origin;

    num_rows_ = ceil(_msg->info.height / grid_resolution_);
    num_cols_ = ceil(_msg->info.width  / grid_resolution_);

    /// Update Grid
    ts_ptr_->SetGrid(std::vector<int>(_msg->data.begin(), _msg->data.end()),
                     num_rows_,
                     num_cols_);
  }
}   // namespace theta_star
