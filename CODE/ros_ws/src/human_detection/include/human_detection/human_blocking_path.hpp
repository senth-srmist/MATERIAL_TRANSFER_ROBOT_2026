#ifndef HUMAN_BLOCKING_PATH_HPP_
#define HUMAN_BLOCKING_PATH_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <zed_interfaces/msg/objects_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace human_detection
{

class HumanBlockingPath : public BT::ConditionNode
{
public:
  HumanBlockingPath(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void humanCallback(
    const zed_interfaces::msg::ObjectsStamped::SharedPtr msg);

  double calculateDistance(double x1, double y1, double x2, double y2);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr sub_;

  std::vector<std::pair<double, double>> humans_;

  std::string human_topic_;
  double distance_threshold_;
  double path_buffer_;
};

}  // namespace human_detection

#endif