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
/// \brief This file defines the tracking_nodes_node class.

#ifndef TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
#define TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_

#include <autoware_auto_msgs/msg/detected_objects.hpp>
#include <autoware_auto_msgs/msg/tracked_objects.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tracking/multi_object_tracker.hpp>
#include <tracking_nodes/visibility_control.hpp>

#include <memory>


namespace autoware
{
namespace tracking_nodes
{

/// \class MultiObjectTrackerNode
/// \brief ROS 2 Node for tracking.
class TRACKING_NODES_PUBLIC MultiObjectTrackerNode : public rclcpp::Node
{
public:
  /// \brief Constructor
  explicit MultiObjectTrackerNode(const rclcpp::NodeOptions & options);

  /// Callback for matching detections + odometry messages.
  /// This unusual signature is mandated by message_filters.
  void process(
    const autoware_auto_msgs::msg::DetectedObjects::ConstSharedPtr & objs,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom);

private:
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<autoware_auto_msgs
      ::msg::DetectedObjects, nav_msgs::msg::Odometry>;
  /// The actual tracker implementation.
  autoware::perception::tracking::MultiObjectTracker m_tracker;
  /// Subscription to odometry and detection messages.
  message_filters::Subscriber<autoware_auto_msgs::msg::DetectedObjects> m_objects_sub;
  message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproximatePolicy>> m_sync;
  /// Publisher for tracked objects.
  rclcpp::Publisher<autoware_auto_msgs::msg::TrackedObjects>::SharedPtr m_pub;
};
}  // namespace tracking_nodes
}  // namespace autoware

#endif  // TRACKING_NODES__MULTI_OBJECT_TRACKER_NODE_HPP_
