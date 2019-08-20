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
 * @file ros_interface_test.cpp
 * @brief Unit tests using Google Test framework to ROS node.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

/// System
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <thread>


/// Project
#include "theta_star/ros_interface.h"

namespace theta_star {
class ThetaStarTest : public ::testing::Test {
 protected:
  /// Pointer to ROS service client
  std::shared_ptr<ros::ServiceClient> srv_client;
  /// Pointer to private node handle
  std::shared_ptr<ros::NodeHandle> p_nh;
  /// Pointer to ROS occupancy grid publisher
  std::shared_ptr<ros::Publisher> grid_pub;
  /// Pointer to ROS interface for Theta Star
  std::shared_ptr<RosInterface> ts_node;
  /// Pointers to different occupancy grids for tests
  std::shared_ptr<std::vector<int8_t>> unoccupied_grid,
                                       funnel_grid,
                                       probabilistic_grid;

  virtual void SetUp() {
    /// Construct private node handle
    p_nh = std::make_shared<ros::NodeHandle>("~");

    /// Construct occupancy grid publisher
    /// Variables to store grid param & topic name
    std::string grid_param = "/theta_star/grid_sub_topic";
    std::string grid_topic;

    /// Retrieve topic to publish grid to
    p_nh->getParam(grid_param.c_str(), grid_topic);
    grid_pub = std::make_shared<ros::Publisher>(
      p_nh->advertise<nav_msgs::OccupancyGrid>(grid_topic, 1));

    /// Construct ROS Interface object using private node handle
    ts_node = std::make_shared<RosInterface>(*p_nh);

    /// Construct service server for nav_msgs::GetPlan service
    srv_client = std::make_shared<ros::ServiceClient>(
      p_nh->serviceClient<nav_msgs::GetPlan>("plan"));

    /// Occupancy Grids
    unoccupied_grid = std::make_shared<std::vector<int8_t>>(
                                                std::initializer_list<int8_t>{
                                                          0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0,
                                                          0, 0, 0, 0, 0, 0});

    funnel_grid = std::make_shared<std::vector<int8_t>>(
                                            std::initializer_list<int8_t>{
                                                      0, 0, 0, 0, 0, 0,
                                                      0, 0, 0, 0, 0, 0,
                                                      0, 0, 1, 0, 1, 0,
                                                      0, 0, 1, 0, 1, 0,
                                                      0, 1, 0, 0, 0, 1,
                                                      1, 0, 0, 0, 0, 0});

    probabilistic_grid = std::make_shared<std::vector<int8_t>>(
                                              std::initializer_list<int8_t>{
                                                   0,  0,  0, 10,  0,  0,
                                                   0,  0,  0, 10,  0,  0,
                                                  10, 10, 20, 20,  0,  0,
                                                  40, 30, 20,  5,  5,  0,
                                                   0,  0,  0,  5,  5,  0,
                                                   0,  0,  0,  0,  0,  0});
  }

  virtual void TearDown() {
    /// Shutdown grid publisher
    grid_pub->shutdown();
    /// Shutdown server client
    srv_client->shutdown();
  }

  /**
   * @brief      Constructs a nav_msgs::OccupancyGrid msg.
   *
   * @param[in]  _start       The start
   * @param[in]  _goal        The goal
   * @param[in]  _origin      The origin
   * @param[in]  _resolution  The resolution
   * @param[in]  _rows        The rows
   * @param[in]  _cols        The cols
   * @param[in]  _grid        The grid
   *
   * @return     The msg.
   */
  nav_msgs::OccupancyGrid InitGrid(const geometry_msgs::PoseStamped& _start,
                               const geometry_msgs::PoseStamped& _goal,
                               const geometry_msgs::Pose& _origin,
                               const double _resolution,
                               const int _rows,
                               const int _cols,
                               const std::vector<int8_t>& _grid) {
    nav_msgs::OccupancyGrid grid;
    /// Header - Stamp
    grid.header.stamp = ros::Time::now();

    /// Info
    grid.info.resolution = _resolution;
    grid.info.width = _cols;
    grid.info.height = _rows;
    grid.info.origin = _origin;

    /// Data
    grid.data = _grid;

    return grid;
  }

  /**
   * @brief      Constructs a geometry_msgs::PoseStamped msg.
   *
   * @param[in]  _x    The x-coordinate
   * @param[in]  _y    The y-coordinate
   * @param[in]  _yaw  The yaw
   *
   * @return     The msg.
   */
  geometry_msgs::PoseStamped InitPoseStamped(const double _x,
                                             const double _y,
                                             const double _yaw) {
      geometry_msgs::PoseStamped ps;

      /// Header - Stamp
      ps.header.stamp = ros::Time::now();

      /// Position
      ps.pose.position.x = _x;
      ps.pose.position.y = _y;
      ps.pose.position.z = 0.0;

      /// Orientation
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, _yaw);
      q.normalize();
      ps.pose.orientation = tf2::toMsg(q);

      return ps;
  }

  /**
   * @brief      Determines if two geometry_msgs::PoseStamed
   *             vectors are the same.
   *
   * @param[in]  _lhs      The left hand side
   * @param[in]  _rhs      The right hand side
   * @param[in]  _epsilon  The epsilon
   *
   * @return     Error string if not equal, empty string otherwise.
   */
  std::string IsSame(const std::vector<geometry_msgs::PoseStamped>& _lhs,
              const std::vector<geometry_msgs::PoseStamped>& _rhs,
              const double _epsilon) {
    std::stringstream msg;

    if (_lhs.size() != _rhs.size()) {
      /// Populate error message - sizes not equal
      msg << "IsSame: _lhs.size() = " << _lhs.size() << "  _rhs.size() = "
        << _rhs.size() << std::endl;
    } else {
      for (int i = 0; i < _lhs.size(); i++) {
        /// Compare each member of message with epsilon for
        /// floating point comparison
        if ((std::fabs(_lhs[i].pose.position.x
              - _rhs[i].pose.position.x) > _epsilon)
        || (std::fabs(_lhs[i].pose.position.y
              - _rhs[i].pose.position.y) > _epsilon)
        || (std::fabs(_lhs[i].pose.position.z
              - _rhs[i].pose.position.z) > _epsilon)
        || (std::fabs(_lhs[i].pose.orientation.x
              - _rhs[i].pose.orientation.x) > _epsilon)
        || (std::fabs(_lhs[i].pose.orientation.y
              - _rhs[i].pose.orientation.y) > _epsilon)
        || (std::fabs(_lhs[i].pose.orientation.z
              - _rhs[i].pose.orientation.z) > _epsilon)
        || (std::fabs(_lhs[i].pose.orientation.w
              - _rhs[i].pose.orientation.w) > _epsilon)) {
          /// Populate error message - value(s) not equal
          msg << "IsSame -- i[" << i << "] -- "
            << "lhs[" << _lhs[i].pose.position.x << " "
            << _lhs[i].pose.position.y << " "
            << _lhs[i].pose.position.z  << " "
            << _lhs[i].pose.orientation.x << " "
            << _lhs[i].pose.orientation.y << " "
            << _lhs[i].pose.orientation.z << " "
            << _lhs[i].pose.orientation.w << " "
            << "]  rhs[" << _rhs[i].pose.position.x << " "
            << _rhs[i].pose.position.y << " "
            << _rhs[i].pose.position.z << " "
            << _rhs[i].pose.orientation.x << " "
            << _rhs[i].pose.orientation.y << " "
            << _rhs[i].pose.orientation.z << " "
            << _rhs[i].pose.orientation.w << "]"
            << std::endl;

          break;
        }
      }
    }
    return msg.str();
  }
};

TEST_F(ThetaStarTest, Unoccupied) {
  /// Declar start, goal, and origin poses
  geometry_msgs::PoseStamped start, goal, origin;

  /// Use copy constructor to assign test params
  origin = InitPoseStamped(3.0, 3.0, 0.0);
  start = InitPoseStamped(1.0, 2.0, -M_PI_2);
  goal = InitPoseStamped(-1.0, -1.0, 0.0);

  int dim = std::sqrt(unoccupied_grid->size());
  nav_msgs::OccupancyGrid grid = InitGrid(start, goal, origin.pose,
                                          1.0, dim, dim,
                                          *unoccupied_grid);

  /// Publish occupancy grid so RosInterface can receive msg
  grid_pub->publish(grid);

  /// Define service request
  nav_msgs::GetPlan srv;
  srv.request.start = start;
  srv.request.goal = goal;

  /// Call service and store response if successful
  nav_msgs::Path path;
  if (srv_client->call(srv) == true)
    path = srv.response.plan;

  /// Optimal Path
  geometry_msgs::PoseStamped goal_with_yaw = InitPoseStamped(-1.0,
                                                             -1.0,
                                                             -2.55359005004222);

  std::vector<geometry_msgs::PoseStamped> optimal_path{start, goal_with_yaw};

  /// Check equality of paths
  std::string msg = IsSame(path.poses, optimal_path, 1e-8);

  EXPECT_TRUE(msg.empty()) << msg;
}
}   //  namespace theta_star

int main(int argc, char **argv) {
  /// Init GTest
  testing::InitGoogleTest(&argc, argv);
  /// Init ROS
  ros::init(argc, argv, "theta_star_test_node");
  /// Have ROS spin on seperate thread to allow for test execution
  std::thread t([]{while(ros::ok()) ros::spin();});
  /// Run tests and store result
  int result =  RUN_ALL_TESTS();
  /// Shutdown ROS
  ros::shutdown();
  /// Join thread
  t.join();

  return result;
}
