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
 * @file ros_interface.h
 * @brief Header file for ROS interface.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

/// System
#include <ros/ros.h>
#include <cmath>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

/// Project
#include "theta_star/theta_star.h"

namespace theta_star {
/**
 * @brief      Class for ROS interface.
 */
class RosInterface {
 public:
  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _p_nh  The p nh
   */
  explicit RosInterface(const ros::NodeHandle& _p_nh);

  /**
   * @brief      Destroys the object.
   */
  ~RosInterface();

 private:
  /**
   * Node Handles
   */
  ros::NodeHandle p_nh_;

  /**
   * Pubs, Subs, and Srvs
   */
  ros::Subscriber grid_sub_;
  ros::ServiceServer plan_srv_server_;

  /**
   * Params
   */
  /// Namespaces & Topics
  std::string node_name_;
  std::string grid_sub_topic_;
  std::string goal_sub_topic_;

  /// Frame IDs
  std::string relative_frame_id_;

  /// Moves & Costs
  std::vector<std::pair<int, int>> moves_;
  std::vector<double> costs_;

  /// Continuous & Discrete Information
  geometry_msgs::Pose grid_origin_;
  double grid_resolution_;
  int num_rows_, num_cols_;

  /**
   * @brief      Loads parameters.
   */
  void LoadParams();

  /**
   * @brief      Initializes all class variables
   */
  void InitVariables();

  /**
   * @brief      Initializes all ROS Publishers, Subscribers, and Services
   */
  void InitPubSubSrv();

  /**
   * @brief      Converts continous pose to discrete coordinate.
   *
   * @param[in]  _pose  The pose
   *
   * @return     The coordinate .
   */
  std::pair<int, int> Cont2Discrete(const geometry_msgs::Pose& _pose);

  /**
   * @brief      Converts discrete coordinates to continous path.
   *
   * @param[in]  _start   The start
   * @param[in]  _coords  The coordinates
   *
   * @return     The path.
   */
  nav_msgs::Path Discrete2Cont(const geometry_msgs::Pose& _start,
    const std::vector<std::pair<int, int>>& _coords);

  /**
   * @brief      Service for path plan.
   *
   * @param[in]  _req  The request
   * @param[in]  _res  The response
   *
   * @return     True if success, False otherwise.
   */
  bool PlanSrv(nav_msgs::GetPlan::Request& _req,
    nav_msgs::GetPlan::Response& _res);

  /**
   * @brief      Callback for occupancy grid.
   *
   * @param[in]  _msg  The message
   */
  void OccupancyGridCB(const nav_msgs::OccupancyGrid::ConstPtr& _msg);

  /**
   * @brief      Verifies ROS parameter exists.
   *
   * @param[in]  _ns        namespace string
   * @param[in]  _param     The parameter
   * @param      _variable  The variable
   *
   * @tparam     T          ROS parameter type
   */
  template<typename T>
  void VerifyParam(const std::string& _ns,
    const std::string& _param,
    T &_variable) {
    if (!p_nh_.getParam(_ns + _param, _variable)) {
      ROS_ERROR("[%s]: Cannot retrieve value for param [%s]. Exiting...",
        ros::this_node::getName().c_str(), _param.c_str());
      exit(1);
    }
  }

  /// Shared Pointers
  std::shared_ptr<ThetaStar> ts_ptr_;
};
}   // namespace theta_star

#endif  // ROS_INTERFACE_H_