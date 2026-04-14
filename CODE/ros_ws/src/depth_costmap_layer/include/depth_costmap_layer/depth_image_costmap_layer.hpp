#pragma once

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "cv_bridge/cv_bridge.h"

namespace depth_costmap_layer
{

class DepthImageCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  DepthImageCostmapLayer();

  void onInitialize() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double *min_x, double *min_y, double *max_x, double *max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override;

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  image_geometry::PinholeCameraModel cam_model_;
  cv::Mat depth_image_;

  bool camera_info_received_;
};

}