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

#ifndef NAV2_DEPTH_OBSTACLE_LAYER__BT_NODES__HUMAN_BLOCKING_PATH_HPP_
#define NAV2_DEPTH_OBSTACLE_LAYER__BT_NODES__HUMAN_BLOCKING_PATH_HPP_

#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#ifdef HAVE_ZED_MSGS
#include "zed_msgs/msg/objects_stamped.hpp"
#endif

namespace nav2_depth_obstacle_layer
{

/**
 * @class HumanBlockingPath
 * @brief BT condition node that returns SUCCESS when a human is blocking the path.
 *
 * Checks if any detected human:
 *   1. Is within `path_width` of any pose on the planned path, AND
 *   2. Is within `human_stop_distance` of the robot's current position
 *
 * This prevents the robot from stopping for humans far away on the path.
 */
class HumanBlockingPath : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor
   * @param name Node name
   * @param config Node configuration
   */
  HumanBlockingPath(
    const std::string & name,
    const BT::NodeConfiguration & config);

  /**
   * @brief Declare BT ports
   * @return Port list
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Execute condition check
   * @return SUCCESS if human blocking, FAILURE otherwise
   */
  BT::NodeStatus tick() override;

private:
#ifdef HAVE_ZED_MSGS
  void humanCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);
#endif

  rclcpp::Node::SharedPtr node_;

#ifdef HAVE_ZED_MSGS
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr sub_;
#endif

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Humans stored in global (map) frame after TF transform
  std::vector<std::pair<double, double>> humans_;
  std::mutex humans_mutex_;

  // Parameters
  std::string human_topic_;
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
  double human_stop_distance_{1.5};
  double path_width_{0.3};
};

}  // namespace nav2_depth_obstacle_layer

#endif  // NAV2_DEPTH_OBSTACLE_LAYER__BT_NODES__HUMAN_BLOCKING_PATH_HPP_
