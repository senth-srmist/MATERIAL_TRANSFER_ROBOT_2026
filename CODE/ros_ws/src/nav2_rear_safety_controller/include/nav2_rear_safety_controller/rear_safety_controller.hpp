#pragma once

#include "nav2_core/controller.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_rear_safety_controller
{

class RearSafetyController : public nav2_core::Controller
{
public:
  RearSafetyController() = default;
  ~RearSafetyController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  std::shared_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  std::shared_ptr<nav2_core::Controller> inner_controller_;

  std::string inner_controller_name_;

  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
  double safe_distance_;
  bool obstacle_detected_;

  // 🔥 NEW (anti-stuck)
  rclcpp::Time last_movement_time_;
  double stuck_timeout_;
  double min_motion_threshold_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

}  // namespace