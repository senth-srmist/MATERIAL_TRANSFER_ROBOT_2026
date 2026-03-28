#include "nav2_rear_safety_controller/rear_safety_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

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

  obstacle_detected_ = false;

  // 🔥 FIX: initialize loader properly
  // 🔥 FIXED loader initialization
loader_ = std::make_shared<pluginlib::ClassLoader<nav2_core::Controller>>(
  "nav2_core",
  "nav2_core::Controller"
);

// 🔥 FIXED usage
inner_controller_ = loader_->createSharedInstance(
  "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");

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

  if (obstacle_detected_ && cmd.twist.linear.x < 0.0)
  {
    cmd.twist.linear.x = 0.0;

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "Rear obstacle detected → Reverse blocked");
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