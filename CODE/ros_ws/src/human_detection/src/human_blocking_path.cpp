#include "human_detection/human_blocking_path.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

namespace human_detection
{

HumanBlockingPath::HumanBlockingPath(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("human_blocking_path_node");

  getInput("human_topic", human_topic_);
  getInput("distance_threshold", distance_threshold_);
  getInput("path_buffer", path_buffer_);

  sub_ = node_->create_subscription<zed_msgs::msg::ObjectsStamped>(
    human_topic_, 10,
    std::bind(&HumanBlockingPath::humanCallback, this, std::placeholders::_1));
}

BT::PortsList HumanBlockingPath::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<double>("distance_threshold", 1.5, "Distance threshold in meters"),
    BT::InputPort<double>("path_buffer", 0.3, "Path intersection buffer in meters"),
    BT::InputPort<std::string>("human_topic", "/zed/zed_node/obj_det/objects", "Human detection topic")
  };
}

void HumanBlockingPath::humanCallback(
  const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
{
  humans_.clear();

  for (const auto & obj : msg->objects) {
    humans_.emplace_back(obj.position[0], obj.position[1]);
  }
}

double HumanBlockingPath::calculateDistance(
  double x1, double y1, double x2, double y2)
{
  return std::hypot(x1 - x2, y1 - y2);
}

BT::NodeStatus HumanBlockingPath::tick()
{
  rclcpp::spin_some(node_);

  nav_msgs::msg::Path path;
  getInput("path", path);

  if (path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  for (const auto & human : humans_) {
    for (const auto & pose : path.poses) {
      double dx = pose.pose.position.x;
      double dy = pose.pose.position.y;

      if (calculateDistance(dx, dy, human.first, human.second) < path_buffer_) {
        double dist = calculateDistance(
          path.poses.front().pose.position.x,
          path.poses.front().pose.position.y,
          human.first, human.second);

        if (dist <= distance_threshold_) {
          return BT::NodeStatus::SUCCESS;
        }
      }
    }
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace human_detection

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<human_detection::HumanBlockingPath>("HumanBlockingPath");
}
