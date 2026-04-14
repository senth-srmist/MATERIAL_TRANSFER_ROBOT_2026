#include "depth_costmap_layer/depth_image_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace depth_costmap_layer
{

DepthImageCostmapLayer::DepthImageCostmapLayer()
: camera_info_received_(false)
{}

void DepthImageCostmapLayer::onInitialize()
{
  auto node = node_.lock();

  depth_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/depth/depth_registered", 10,
    std::bind(&DepthImageCostmapLayer::depthCallback, this, std::placeholders::_1));

  info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/zed/zed_node/depth/camera_info", 10,
    std::bind(&DepthImageCostmapLayer::cameraInfoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "DepthImageCostmapLayer initialized");
}

void DepthImageCostmapLayer::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  cam_model_.fromCameraInfo(msg);
  camera_info_received_ = true;
}

void DepthImageCostmapLayer::depthCallback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  try
  {
    depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("depth_layer"), "CV Bridge error: %s", e.what());
  }
}

void DepthImageCostmapLayer::updateBounds(
  double, double, double,
  double *min_x, double *min_y, double *max_x, double *max_y)
{
  *min_x = -10.0;
  *min_y = -10.0;
  *max_x = 10.0;
  *max_y = 10.0;
}

void DepthImageCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  // 🔕 Suppress unused warnings
  (void)min_i;
  (void)min_j;
  (void)max_i;
  (void)max_j;

  if (!camera_info_received_ || depth_image_.empty())
    return;

  for (int v = 0; v < depth_image_.rows; v += 10)
  {
    for (int u = 0; u < depth_image_.cols; u += 10)
    {
      float d = depth_image_.at<float>(v, u);

      if (std::isnan(d) || std::isinf(d))
        continue;

      if (d > 0.3 && d < 3.0)
      {
        double x = (u - cam_model_.cx()) * d / cam_model_.fx();
        double y = (v - cam_model_.cy()) * d / cam_model_.fy();

        unsigned int mx, my;
        if (master_grid.worldToMap(x, y, mx, my))
        {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

void DepthImageCostmapLayer::reset()
{
  RCLCPP_INFO(rclcpp::get_logger("depth_layer"), "Depth layer reset");
}

bool DepthImageCostmapLayer::isClearable()
{
  return true;
}

} // namespace depth_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  depth_costmap_layer::DepthImageCostmapLayer,
  nav2_costmap_2d::Layer)