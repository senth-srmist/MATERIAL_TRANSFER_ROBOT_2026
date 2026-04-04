#ifndef WAIT_HUMAN_CLEARS_HPP_
#define WAIT_HUMAN_CLEARS_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>
#include <string>
#include <cmath>

namespace human_detection
{

class WaitUntilHumanClears : public BT::StatefulActionNode
{
public:
  WaitUntilHumanClears(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void humanCallback(
    const zed_msgs::msg::ObjectsStamped::SharedPtr msg);

  bool isBlocking(const nav_msgs::msg::Path & path);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alarm_pub_;

  std::vector<std::pair<double, double>> humans_;

  std::string human_topic_{"/zed/zed_node/obj_det/objects"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string alarm_topic_{"/human_alarm/active"};

  double distance_threshold_{1.5};
};

}  // namespace human_detection

#endif  // WAIT_HUMAN_CLEARS_HPP_
