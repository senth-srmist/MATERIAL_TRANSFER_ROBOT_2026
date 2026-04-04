#include "human_detection/wait_human_clears.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

namespace human_detection
{

WaitUntilHumanClears::WaitUntilHumanClears(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("wait_human_clears_node");

  getInput("human_topic", human_topic_);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("alarm_topic", alarm_topic_);
  getInput("distance_threshold", distance_threshold_);

  sub_ = node_->create_subscription<zed_interfaces::msg::ObjectsStamped>(
    human_topic_, 10,
    std::bind(&WaitUntilHumanClears::humanCallback, this, std::placeholders::_1));

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  alarm_pub_ = node_->create_publisher<std_msgs::msg::Bool>(alarm_topic_, 10);
}

BT::PortsList WaitUntilHumanClears::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<double>("distance_threshold", 1.5),
    BT::InputPort<std::string>("human_topic", "/zed/obj_det/objects"),
    BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel"),
    BT::InputPort<std::string>("alarm_topic", "/human_alarm/active")
  };
}

void WaitUntilHumanClears::humanCallback(
  const zed_interfaces::msg::ObjectsStamped::SharedPtr msg)
{
  humans_.clear();

  for (const auto & obj : msg->objects)
  {
    if (obj.label != "Person") continue;
    humans_.emplace_back(obj.position[0], obj.position[1]);
  }
}

bool WaitUntilHumanClears::isBlocking(const nav_msgs::msg::Path & path)
{
  for (const auto & human : humans_)
  {
    for (const auto & pose : path.poses)
    {
      double dx = pose.pose.position.x;
      double dy = pose.pose.position.y;

      if (std::hypot(dx - human.first, dy - human.second) < 0.3)
      {
        if (std::hypot(
          path.poses.front().pose.position.x - human.first,
          path.poses.front().pose.position.y - human.second)
          <= distance_threshold_)
        {
          return true;
        }
      }
    }
  }
  return false;
}

BT::NodeStatus WaitUntilHumanClears::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilHumanClears::onRunning()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  if (isBlocking(path))
  {
    geometry_msgs::msg::Twist stop;
    vel_pub_->publish(stop);
    return BT::NodeStatus::RUNNING;
  }

  std_msgs::msg::Bool msg;
  msg.data = false;
  alarm_pub_->publish(msg);

  return BT::NodeStatus::SUCCESS;
}

void WaitUntilHumanClears::onHalted() {}

}  // namespace human_detection

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<human_detection::WaitUntilHumanClears>("WaitUntilHumanClears");
}