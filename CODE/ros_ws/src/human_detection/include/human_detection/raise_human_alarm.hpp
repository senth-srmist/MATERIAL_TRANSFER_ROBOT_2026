#ifndef RAISE_HUMAN_ALARM_HPP_
#define RAISE_HUMAN_ALARM_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace human_detection
{

class RaiseHumanAlarm : public BT::SyncActionNode
{
public:
  RaiseHumanAlarm(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

  std::string alarm_topic_;
};

}  // namespace human_detection

#endif