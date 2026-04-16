// Copyright (c) 2026 Tejas
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_depth_obstacle_layer/bt_nodes/human_blocking_path.hpp"

#include <cmath>

namespace nav2_depth_obstacle_layer
{

HumanBlockingPath::HumanBlockingPath(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("human_blocking_path_bt_node");

  // Try to get params from costmap layer namespace (local_costmap.depth_obstacle_layer.*)
  // This allows configuration in nav2_params.yaml alongside the costmap layer
  const std::string ns = "local_costmap.local_costmap.depth_obstacle_layer";

  node_->declare_parameter(ns + ".human_stop_distance", 1.5);
  node_->declare_parameter(ns + ".path_width", 0.3);
  node_->declare_parameter(ns + ".human_topic", "/zed/zed_node/obj_det/objects");
  node_->declare_parameter(ns + ".global_frame", "map");
  node_->declare_parameter(ns + ".robot_frame", "base_link");

  human_stop_distance_ = node_->get_parameter(ns + ".human_stop_distance").as_double();
  path_width_ = node_->get_parameter(ns + ".path_width").as_double();
  human_topic_ = node_->get_parameter(ns + ".human_topic").as_string();
  global_frame_ = node_->get_parameter(ns + ".global_frame").as_string();
  robot_frame_ = node_->get_parameter(ns + ".robot_frame").as_string();

  // BT port overrides (if specified in BT XML, takes priority)
  double bt_val;
  std::string bt_str;
  if (getInput("human_stop_distance", bt_val)) { human_stop_distance_ = bt_val; }
  if (getInput("path_width", bt_val)) { path_width_ = bt_val; }
  if (getInput("human_topic", bt_str) && !bt_str.empty()) { human_topic_ = bt_str; }
  if (getInput("global_frame", bt_str) && !bt_str.empty()) { global_frame_ = bt_str; }
  if (getInput("robot_frame", bt_str) && !bt_str.empty()) { robot_frame_ = bt_str; }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

#ifdef HAVE_ZED_MSGS
  sub_ = node_->create_subscription<zed_msgs::msg::ObjectsStamped>(
    human_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&HumanBlockingPath::humanCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "HumanBlockingPath: human_stop_distance=%.2fm, path_width=%.2fm, topic='%s'",
    human_stop_distance_, path_width_, human_topic_.c_str());
#else
  RCLCPP_ERROR(
    node_->get_logger(),
    "HumanBlockingPath: built without zed_msgs support");
#endif
}

BT::PortsList HumanBlockingPath::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Current navigation path"),
    BT::InputPort<double>(
      "human_stop_distance", 1.5,
      "Only stop if human closer than this (meters)"),
    BT::InputPort<double>(
      "path_width", 0.3,
      "Human must be within this distance of path (meters)"),
    BT::InputPort<std::string>(
      "human_topic", "/zed/zed_node/obj_det/objects",
      "Human detection topic"),
    BT::InputPort<std::string>("global_frame", "map", "Global planning frame"),
    BT::InputPort<std::string>("robot_frame", "base_link", "Robot base frame"),
  };
}

#ifdef HAVE_ZED_MSGS
void HumanBlockingPath::humanCallback(
  const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(humans_mutex_);
  humans_.clear();
  humans_.reserve(msg->objects.size());

  for (const auto & obj : msg->objects) {
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header = msg->header;
    pt_in.point.x = obj.position[0];
    pt_in.point.y = obj.position[1];
    pt_in.point.z = obj.position[2];

    try {
      tf_buffer_->transform(pt_in, pt_out, global_frame_, tf2::durationFromSec(0.1));
      humans_.emplace_back(pt_out.point.x, pt_out.point.y);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "TF failed for human detection: %s", ex.what());
    }
  }
}
#endif

BT::NodeStatus HumanBlockingPath::tick()
{
  rclcpp::spin_some(node_);

#ifndef HAVE_ZED_MSGS
  return BT::NodeStatus::FAILURE;
#else

  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // Get robot position in global frame
  geometry_msgs::msg::TransformStamped robot_tf;
  try {
    robot_tf = tf_buffer_->lookupTransform(
      global_frame_, robot_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Robot TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = robot_tf.transform.translation.x;
  const double robot_y = robot_tf.transform.translation.y;
  const double stop_dist_sq = human_stop_distance_ * human_stop_distance_;
  const double path_width_sq = path_width_ * path_width_;

  // Thread-safe copy
  std::vector<std::pair<double, double>> humans_copy;
  {
    std::lock_guard<std::mutex> lock(humans_mutex_);
    humans_copy = humans_;
  }

  for (const auto & human : humans_copy) {
    // Check 1: Is human close enough to robot?
    const double dx_robot = human.first - robot_x;
    const double dy_robot = human.second - robot_y;
    const double dist_to_robot_sq = dx_robot * dx_robot + dy_robot * dy_robot;

    if (dist_to_robot_sq > stop_dist_sq) {
      continue;  // Human too far from robot
    }

    // Check 2: Is human on/near the path?
    for (const auto & pose : path.poses) {
      const double dx_path = pose.pose.position.x - human.first;
      const double dy_path = pose.pose.position.y - human.second;
      const double dist_to_path_sq = dx_path * dx_path + dy_path * dy_path;

      if (dist_to_path_sq < path_width_sq) {
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Human blocking at (%.2f,%.2f), %.2fm from robot",
          human.first, human.second, std::sqrt(dist_to_robot_sq));
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  return BT::NodeStatus::FAILURE;
#endif
}

}  // namespace nav2_depth_obstacle_layer
