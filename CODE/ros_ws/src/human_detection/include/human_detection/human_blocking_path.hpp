#ifndef HUMAN_BLOCKING_PATH_HPP_
#define HUMAN_BLOCKING_PATH_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <zed_msgs/msg/objects_stamped.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace human_detection {

class HumanBlockingPath : public BT::ConditionNode {
public:
  HumanBlockingPath(const std::string &name,
                    const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void humanCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);

  double calculateDistance(double x1, double y1, double x2, double y2);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // humans_ stored in global (map) frame after transform
  std::vector<std::pair<double, double>> humans_;

  std::string human_topic_;
  std::string global_frame_{"map"};
  double distance_threshold_{1.5};
  double path_buffer_{0.3};
};

} // namespace human_detection

#endif // HUMAN_BLOCKING_PATH_HPP_
