#pragma once

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace depth_costmap_layer {

class DepthImageCostmapLayer : public nav2_costmap_2d::Layer {
public:
  DepthImageCostmapLayer();

  void onInitialize() override;

  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double *min_x, double *min_y, double *max_x,
                    double *max_y) override;

  void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i,
                   int min_j, int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override;

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  image_geometry::PinholeCameraModel cam_model_;
  cv::Mat depth_image_;
  std::mutex depth_mutex_;

  bool camera_info_received_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  int stride_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double obstacle_range_;
  std::string global_frame_;
  std::string ground_frame_;
  std::string height_frame_;

  // Timestamp of last depth image
  rclcpp::Time last_depth_stamp_;
};

} // namespace depth_costmap_layer
