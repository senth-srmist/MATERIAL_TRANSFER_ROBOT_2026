#include "human_detection/wait_human_clears.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace human_detection {

WaitUntilHumanClears::WaitUntilHumanClears(const std::string &name,
                                           const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config) {
  node_ = rclcpp::Node::make_shared("wait_human_clears_node");

  getInput("human_topic", human_topic_);
  getInput("cmd_vel_topic", cmd_vel_topic_);
  getInput("alarm_topic", alarm_topic_);
  getInput("distance_threshold", distance_threshold_);
  getInput("global_frame", global_frame_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = node_->create_subscription<zed_msgs::msg::ObjectsStamped>(
      human_topic_, 10,
      std::bind(&WaitUntilHumanClears::humanCallback, this,
                std::placeholders::_1));

  vel_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  alarm_pub_ = node_->create_publisher<std_msgs::msg::Bool>(alarm_topic_, 10);
}

BT::PortsList WaitUntilHumanClears::providedPorts() {
  return {BT::InputPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<double>("distance_threshold", 1.5,
                                "Distance threshold in meters"),
          BT::InputPort<std::string>("human_topic",
                                     "/zed/zed_node/obj_det/objects",
                                     "Human detection topic"),
          BT::InputPort<std::string>("cmd_vel_topic", "/cmd_vel",
                                     "Velocity command topic"),
          BT::InputPort<std::string>("alarm_topic", "/human_alarm/active",
                                     "Alarm topic"),
          BT::InputPort<std::string>("global_frame", "map",
                                     "Global planning frame")};
}

void WaitUntilHumanClears::humanCallback(
    const zed_msgs::msg::ObjectsStamped::SharedPtr msg) {
  humans_.clear();

  for (const auto &obj : msg->objects) {
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header = msg->header;
    pt_in.point.x = obj.position[0];
    pt_in.point.y = obj.position[1];
    pt_in.point.z = obj.position[2];

    try {
      tf_buffer_->transform(pt_in, pt_out, global_frame_,
                            tf2::durationFromSec(0.3));
      humans_.emplace_back(pt_out.point.x, pt_out.point.y);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node_->get_logger(), "WaitUntilHumanClears TF failed: %s",
                  ex.what());
    }
  }
}

bool WaitUntilHumanClears::isBlocking(const nav_msgs::msg::Path &path) {
  for (const auto &human : humans_) {
    for (const auto &pose : path.poses) {
      double dx = pose.pose.position.x;
      double dy = pose.pose.position.y;

      if (std::hypot(dx - human.first, dy - human.second) < 0.3) {
        if (std::hypot(path.poses.front().pose.position.x - human.first,
                       path.poses.front().pose.position.y - human.second) <=
            distance_threshold_) {
          return true;
        }
      }
    }
  }
  return false;
}

BT::NodeStatus WaitUntilHumanClears::onStart() {
  std_msgs::msg::Bool alarm_msg;
  alarm_msg.data = true;
  alarm_pub_->publish(alarm_msg);

  // Stop the robot immediately
  geometry_msgs::msg::Twist stop;
  vel_pub_->publish(stop);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitUntilHumanClears::onRunning() {
  rclcpp::spin_some(node_);

  nav_msgs::msg::Path path;
  getInput("path", path);

  if (isBlocking(path)) {
    // Human still blocking — keep stopped, keep alarm on
    geometry_msgs::msg::Twist stop;
    vel_pub_->publish(stop);
    return BT::NodeStatus::RUNNING;
  }

  // Human cleared — lower the alarm
  std_msgs::msg::Bool alarm_msg;
  alarm_msg.data = false;
  alarm_pub_->publish(alarm_msg);

  // Return SUCCESS so the parent Sequence succeeds, the Inverter turns it
  // to FAILURE, and the ReactiveSequence restarts FollowPath next tick.
  return BT::NodeStatus::SUCCESS;
}

void WaitUntilHumanClears::onHalted() {
  geometry_msgs::msg::Twist stop;
  vel_pub_->publish(stop);
}

} // namespace human_detection
