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
 * @file ros_interface_node.cpp
 * @brief Node for theta_star ROS interface.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <ros/ros.h>

#include "theta_star/ros_interface.h"

int main(int argc, char *argv[]) {
  /// Initialize ROS
  ros::init(argc, argv, "theta_star_node");

  /// Construct private node handle
  ros::NodeHandle p_nh("~");

  /// Construct ROS Interface object using private node handle
  theta_star::RosInterface g_node(p_nh);

  /// Allow ROS thread to spin
  ros::spin();

  return 0;
}
