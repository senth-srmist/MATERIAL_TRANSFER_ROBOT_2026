// Copyright (c) 2026 Tejas
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_DEPTH_OBSTACLE_LAYER__DEPTH_OBSTACLE_LAYER_HPP_
#define NAV2_DEPTH_OBSTACLE_LAYER__DEPTH_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "image_geometry/pinhole_camera_model.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Optional: ZED human detection support
#ifdef HAVE_ZED_MSGS
#include "zed_msgs/msg/objects_stamped.hpp"
#endif

namespace nav2_depth_obstacle_layer
{

/**
 * @class DepthObstacleLayer
 * @brief Nav2 costmap layer that projects depth image points as obstacles.
 *
 * This layer processes depth images and projects valid points onto the costmap
 * as LETHAL_OBSTACLE costs. Points are filtered by height above ground to
 * exclude floor points and overhead obstacles.
 *
 * Optionally supports human detection masking: when enabled, regions containing
 * detected humans are excluded from obstacle marking. This allows behavior tree
 * nodes to handle human encounters (stop and wait) without triggering path
 * replanning.
 *
 * Parameters:
 * - depth_topic: Depth image topic (default: /depth/image_rect)
 * - camera_info_topic: Camera info topic (default: /depth/camera_info)
 * - ground_frame: Frame for height calculation (default: base_link)
 * - stride: Pixel stride for processing (default: 2)
 * - obstacle_range: Maximum range in meters (default: 3.0)
 * - min_obstacle_height: Minimum height above ground (default: 0.05)
 * - max_obstacle_height: Maximum height above ground (default: 1.5)
 * - human_mask_enabled: Enable human detection masking (default: false)
 * - human_topic: Human detection topic (default: "")
 * - human_mask_padding: Padding around human bbox in pixels (default: 20)
 * - human_persistence: Time to keep human detection in seconds (default: 0.5)
 */
class DepthObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
   * @brief Constructor
   */
  DepthObstacleLayer();

  /**
   * @brief Destructor
   */
  ~DepthObstacleLayer() override = default;

  /**
   * @brief Initialize the layer
   */
  void onInitialize() override;

  /**
   * @brief Update the bounds of the master costmap
   * @param robot_x X position of robot
   * @param robot_y Y position of robot
   * @param robot_yaw Yaw of robot
   * @param min_x Minimum x bound to update
   * @param min_y Minimum y bound to update
   * @param max_x Maximum x bound to update
   * @param max_y Maximum y bound to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap
   * @param master_grid Reference to master costmap
   * @param min_i Minimum x cell index
   * @param min_j Minimum y cell index
   * @param max_i Maximum x cell index
   * @param max_j Maximum y cell index
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Reset the layer
   */
  void reset() override;

  /**
   * @brief Check if layer is clearable
   * @return True if clearable
   */
  bool isClearable() override;

private:
  /**
   * @brief Bounding box in image pixel coordinates
   */
  struct ImageBBox
  {
    int u_min;
    int u_max;
    int v_min;
    int v_max;
    rclcpp::Time stamp;
  };

  /**
   * @brief Callback for depth images
   * @param msg Depth image message
   */
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Callback for camera info
   * @param msg Camera info message
   */
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

#ifdef HAVE_ZED_MSGS
  /**
   * @brief Callback for human detections
   * @param msg Objects message from ZED
   */
  void humanCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);
#endif

  /**
   * @brief Remove stale human detections based on persistence time
   */
  void removeStaleHumans();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

#ifdef HAVE_ZED_MSGS
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr human_sub_;
#endif

  // Camera model (protected by depth_mutex_)
  image_geometry::PinholeCameraModel cam_model_;
  bool camera_info_received_{false};

  // Depth image (mutex protected)
  cv::Mat depth_image_;
  cv::Mat depth_buffer_;  // Pre-allocated buffer for swap pattern
  std::mutex depth_mutex_;
  rclcpp::Time last_depth_stamp_;

  // Human bounding boxes in image space (mutex protected)
  std::vector<ImageBBox> human_bboxes_;
  mutable std::mutex human_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string human_topic_;
  std::string global_frame_;
  std::string ground_frame_;

  int stride_;
  double obstacle_range_;
  double obstacle_range_sq_;  // Cached squared value for fast comparison
  double min_obstacle_height_;
  double max_obstacle_height_;

  bool human_mask_enabled_;
  int human_mask_padding_;
  double human_persistence_;
};

}  // namespace nav2_depth_obstacle_layer

#endif  // NAV2_DEPTH_OBSTACLE_LAYER__DEPTH_OBSTACLE_LAYER_HPP_
