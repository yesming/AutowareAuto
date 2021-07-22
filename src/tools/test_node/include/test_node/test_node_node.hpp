// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the test_node_node class.

#ifndef TEST_NODE__TEST_NODE_NODE_HPP_
#define TEST_NODE__TEST_NODE_NODE_HPP_

#include <test_node/test_node.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace test_node
{

/// \class TestNodeNode
/// \brief ROS 2 Node for hello world.
class TEST_NODE_PUBLIC TestNodeNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit TestNodeNode(const rclcpp::NodeOptions & options);

  void process(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);

private:
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> m_pt1_sub;
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> m_pt2_sub;
  message_filters::Cache<geometry_msgs::msg::PointStamped> m_header_cache;
  bool verbose;  ///< whether to use verbose output or not.
};
}  // namespace test_node
}  // namespace autoware

#endif  // TEST_NODE__TEST_NODE_NODE_HPP_
