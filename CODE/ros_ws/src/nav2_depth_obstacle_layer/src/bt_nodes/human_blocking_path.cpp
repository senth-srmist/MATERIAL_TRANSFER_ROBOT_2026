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
  // Use the shared bt_navigator node — avoids rogue nodes and spin_some-in-tick.
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Read configuration from ROS2 parameters (declared in bt_navigator section of nav2_params.yaml).
  // Use has_parameter() guard because both BT nodes share the same node instance.
  auto init_param = [&](const std::string & pname, auto default_val) {
    if (!node_->has_parameter(pname)) {
      node_->declare_parameter(pname, default_val);
    }
  };

  init_param("human_stop_distance", 0.5);
  node_->get_parameter("human_stop_distance", human_stop_distance_);

  init_param("path_width", 0.5);
  node_->get_parameter("path_width", path_width_);

  init_param("human_topic", std::string("/zed/zed_node/obj_det/objects"));
  node_->get_parameter("human_topic", human_topic_);

  init_param("global_frame", std::string("map"));
  node_->get_parameter("global_frame", global_frame_);

  init_param("robot_frame", std::string("base_link"));
  node_->get_parameter("robot_frame", robot_frame_);

  init_param("human_persistence", 0.5);
  node_->get_parameter("human_persistence", human_persistence_);

  last_person_stamp_ = node_->now();

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
  };
}

#ifdef HAVE_ZED_MSGS
void HumanBlockingPath::humanCallback(
  const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
{
  // Skip clear+rebuild when no persons — preserve last detections for persistence window
  bool has_person = false;
  for (const auto & obj : msg->objects) {
    if (obj.label == "Person") { has_person = true; break; }
  }
  if (!has_person) {
    // Expire detections only after the persistence window lapses
    if ((node_->now() - last_person_stamp_).seconds() > human_persistence_) {
      std::lock_guard<std::mutex> lock(humans_mutex_);
      humans_.clear();
    }
    return;
  }

  std::lock_guard<std::mutex> lock(humans_mutex_);
  humans_.clear();
  humans_.reserve(msg->objects.size());
  last_person_stamp_ = node_->now();

  for (const auto & obj : msg->objects) {
    if (obj.label != "Person") {
      continue;
    }
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
  // No spin_some — the shared bt_navigator executor services this node's callbacks.

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
