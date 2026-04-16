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

#ifndef NAV2_DEPTH_OBSTACLE_LAYER__BT_NODES__WAIT_UNTIL_HUMAN_CLEARS_HPP_
#define NAV2_DEPTH_OBSTACLE_LAYER__BT_NODES__WAIT_UNTIL_HUMAN_CLEARS_HPP_

#include <chrono>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "alerts_system/srv/speaker_service.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#ifdef HAVE_ZED_MSGS
#include "zed_msgs/msg/objects_stamped.hpp"
#endif

namespace nav2_depth_obstacle_layer {

/**
 * @class WaitUntilHumanClears
 * @brief BT action node that stops robot and waits until human clears the path.
 *
 * On start:
 *   - Publishes zero velocity to stop robot
 *   - Calls /speak service with configured message
 *
 * While running:
 *   - Keeps publishing zero velocity
 *   - Checks if human still blocking
 *   - Periodically re-announces if still blocked
 *
 * Returns SUCCESS when human clears, allowing BT to resume navigation.
 */
class WaitUntilHumanClears : public BT::StatefulActionNode {
public:
  /**
   * @brief Constructor
   * @param name Node name
   * @param config Node configuration
   */
  WaitUntilHumanClears(const std::string &name,
                       const BT::NodeConfiguration &config);

  /**
   * @brief Declare BT ports
   * @return Port list
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Called when node starts
   * @return RUNNING to continue, SUCCESS/FAILURE to finish
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called while node is running
   * @return RUNNING to continue, SUCCESS when human clears
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Called when node is halted
   */
  void onHalted() override;

private:
#ifdef HAVE_ZED_MSGS
  void humanCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);
#endif

  bool isHumanBlocking(const nav_msgs::msg::Path &path);
  void speak(const std::string &message);
  void stopRobot();

  rclcpp::Node::SharedPtr node_;

#ifdef HAVE_ZED_MSGS
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr sub_;
#endif

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Client<alerts_system::srv::SpeakerService>::SharedPtr speak_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Humans in global frame
  std::vector<std::pair<double, double>> humans_;
  std::mutex humans_mutex_;

  // Parameters
  std::string human_topic_;
  std::string cmd_vel_topic_;
  std::string speak_service_;
  std::string speak_message_;
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
  double human_stop_distance_{1.5};
  double path_width_{0.3};
  double speak_interval_{5.0};

  // State
  rclcpp::Time last_speak_time_;
};

} // namespace nav2_depth_obstacle_layer

#endif // NAV2_DEPTH_OBSTACLE_LAYER__BT_NODES__WAIT_UNTIL_HUMAN_CLEARS_HPP_
