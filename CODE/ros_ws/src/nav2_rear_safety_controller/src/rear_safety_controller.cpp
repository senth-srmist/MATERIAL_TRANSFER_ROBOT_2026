#include "nav2_rear_safety_controller/rear_safety_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>

namespace nav2_rear_safety_controller
{

void RearSafetyController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent.lock();

  safe_distance_ = node_->declare_parameter(name + ".safe_distance", 0.5);
  inner_controller_name_ = node_->declare_parameter(name + ".inner_controller", "FollowPath");

  // 🔥 NEW PARAMETERS
  stuck_timeout_ = node_->declare_parameter(name + ".stuck_timeout", 3.0);
  min_motion_threshold_ = node_->declare_parameter(name + ".min_motion_threshold", 0.05);

  last_movement_time_ = node_->now();
  obstacle_detected_ = false;

  // Load inner controller (RPP)
  loader_ = std::make_shared<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core",
    "nav2_core::Controller"
  );

  inner_controller_ = loader_->createSharedInstance(
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");

  inner_controller_->configure(parent, inner_controller_name_, nullptr, nullptr);

  // TF Mini subscriber
  range_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    "/range_rear", 10,
    [this](sensor_msgs::msg::Range::SharedPtr msg)
    {
      obstacle_detected_ = (msg->range < safe_distance_);
    });

  RCLCPP_INFO(node_->get_logger(), "Rear Safety Controller loaded successfully");
}

void RearSafetyController::activate()
{
  inner_controller_->activate();
}

void RearSafetyController::deactivate()
{
  inner_controller_->deactivate();
}

void RearSafetyController::cleanup()
{
  inner_controller_->cleanup();
}

void RearSafetyController::setPlan(const nav_msgs::msg::Path & path)
{
  inner_controller_->setPlan(path);
}

geometry_msgs::msg::TwistStamped
RearSafetyController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd = inner_controller_->computeVelocityCommands(pose, velocity, goal_checker);

  // ===============================
  // 🔥 REAR SAFETY + ESCAPE LOGIC
  // ===============================
  if (obstacle_detected_ && cmd.twist.linear.x < 0.0)
  {
    cmd.twist.linear.x = 0.0;

    // Allow turning
    if (std::fabs(cmd.twist.angular.z) < 0.2)
    {
      cmd.twist.angular.z = 0.5;
    }

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "Rear blocked → rotating to escape");
  }

  // ===============================
  // 🔥 STUCK DETECTION
  // ===============================
  bool is_moving =
    std::fabs(cmd.twist.linear.x) > min_motion_threshold_ ||
    std::fabs(cmd.twist.angular.z) > min_motion_threshold_;

  if (is_moving)
  {
    last_movement_time_ = node_->now();
  }
  else
  {
    double stuck_time = (node_->now() - last_movement_time_).seconds();

    if (stuck_time > stuck_timeout_)
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "Robot stuck for %.2f sec → forcing rotation",
        stuck_time);

      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.6;
    }
  }

  return cmd;
}

void RearSafetyController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  inner_controller_->setSpeedLimit(speed_limit, percentage);
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(
  nav2_rear_safety_controller::RearSafetyController,
  nav2_core::Controller)