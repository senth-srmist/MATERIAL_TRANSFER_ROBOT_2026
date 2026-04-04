#include "human_detection/raise_human_alarm.hpp"

namespace human_detection {

RaiseHumanAlarm::RaiseHumanAlarm(const std::string &name,
                                 const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {
  node_ = rclcpp::Node::make_shared("raise_human_alarm_node");

  getInput("alarm_topic", alarm_topic_);

  pub_ = node_->create_publisher<std_msgs::msg::Bool>(alarm_topic_, 10);
}

BT::PortsList RaiseHumanAlarm::providedPorts() {
  return {BT::InputPort<std::string>("alarm_topic", "/human_alarm/active",
                                     "Alarm status topic")};
}

BT::NodeStatus RaiseHumanAlarm::tick() {
  std_msgs::msg::Bool msg;
  msg.data = true;
  pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

} // namespace human_detection
