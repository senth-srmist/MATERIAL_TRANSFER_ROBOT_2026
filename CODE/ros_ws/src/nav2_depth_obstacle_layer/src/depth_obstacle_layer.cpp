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

#include "nav2_depth_obstacle_layer/depth_obstacle_layer.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Vector3.h"

namespace nav2_depth_obstacle_layer {

DepthObstacleLayer::DepthObstacleLayer()
    : camera_info_received_(false), obstacle_range_(3.0),
      obstacle_range_sq_(9.0) {}

void DepthObstacleLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    RCLCPP_FATAL(rclcpp::get_logger("nav2_depth_obstacle_layer"),
                 "Failed to lock node pointer during initialization");
    throw std::runtime_error("Failed to lock node");
  }

  // Declare parameters with defaults
  declareParameter("depth_topic", rclcpp::ParameterValue("/depth/image_rect"));
  declareParameter("camera_info_topic",
                   rclcpp::ParameterValue("/depth/camera_info"));
  declareParameter("ground_frame", rclcpp::ParameterValue("base_link"));
  declareParameter("stride", rclcpp::ParameterValue(2));
  declareParameter("obstacle_range", rclcpp::ParameterValue(3.0));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.05));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(1.5));
  declareParameter("human_mask_enabled", rclcpp::ParameterValue(false));
  declareParameter("human_topic", rclcpp::ParameterValue(""));
  declareParameter("human_mask_padding", rclcpp::ParameterValue(20));
  declareParameter("human_persistence", rclcpp::ParameterValue(0.5));
  // BT node params (declared here so they're in same namespace, read by BT
  // nodes)
  declareParameter("human_stop_distance", rclcpp::ParameterValue(1.5));
  declareParameter("path_width", rclcpp::ParameterValue(0.3));
  declareParameter("speak_message",
                   rclcpp::ParameterValue("Please move away from the robot"));
  declareParameter("speak_interval", rclcpp::ParameterValue(5.0));

  // Get parameters using layer name prefix
  node->get_parameter(name_ + ".depth_topic", depth_topic_);
  node->get_parameter(name_ + ".camera_info_topic", camera_info_topic_);
  node->get_parameter(name_ + ".ground_frame", ground_frame_);
  node->get_parameter(name_ + ".stride", stride_);
  node->get_parameter(name_ + ".obstacle_range", obstacle_range_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".human_mask_enabled", human_mask_enabled_);
  node->get_parameter(name_ + ".human_topic", human_topic_);
  node->get_parameter(name_ + ".human_mask_padding", human_mask_padding_);
  node->get_parameter(name_ + ".human_persistence", human_persistence_);

  // Validate parameters
  if (stride_ < 1) {
    RCLCPP_WARN(node->get_logger(),
                "DepthObstacleLayer: stride must be >= 1, got %d, setting to 1",
                stride_);
    stride_ = 1;
  }

  if (obstacle_range_ <= 0.0) {
    RCLCPP_WARN(node->get_logger(),
                "DepthObstacleLayer: obstacle_range must be > 0, got %.2f, "
                "setting to 3.0",
                obstacle_range_);
    obstacle_range_ = 3.0;
  }
  obstacle_range_sq_ = obstacle_range_ * obstacle_range_;

  if (min_obstacle_height_ >= max_obstacle_height_) {
    RCLCPP_WARN(node->get_logger(),
                "DepthObstacleLayer: min_obstacle_height (%.2f) must be < "
                "max_obstacle_height (%.2f), "
                "using defaults [0.05, 1.5]",
                min_obstacle_height_, max_obstacle_height_);
    min_obstacle_height_ = 0.05;
    max_obstacle_height_ = 1.5;
  }

  if (human_mask_padding_ < 0) {
    RCLCPP_WARN(node->get_logger(),
                "DepthObstacleLayer: human_mask_padding must be >= 0, got %d, "
                "setting to 0",
                human_mask_padding_);
    human_mask_padding_ = 0;
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Depth image subscriber with SensorDataQoS (best effort, volatile)
  depth_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleLayer::depthCallback, this,
                std::placeholders::_1));

  // Camera info subscriber
  info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&DepthObstacleLayer::cameraInfoCallback, this,
                std::placeholders::_1));

#ifdef HAVE_ZED_MSGS
  // Human detection subscriber (optional)
  if (human_mask_enabled_ && !human_topic_.empty()) {
    human_sub_ = node->create_subscription<zed_msgs::msg::ObjectsStamped>(
        human_topic_, rclcpp::SensorDataQoS(),
        std::bind(&DepthObstacleLayer::humanCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(node->get_logger(),
                "DepthObstacleLayer: human masking enabled, topic='%s', "
                "padding=%dpx, persistence=%.2fs",
                human_topic_.c_str(), human_mask_padding_, human_persistence_);
  } else if (human_mask_enabled_) {
    RCLCPP_WARN(node->get_logger(),
                "DepthObstacleLayer: human_mask_enabled=true but human_topic "
                "is empty, disabling");
    human_mask_enabled_ = false;
  }
#else
  if (human_mask_enabled_) {
    RCLCPP_ERROR(node->get_logger(),
                 "DepthObstacleLayer: human_mask_enabled=true but built "
                 "without zed_msgs support");
    human_mask_enabled_ = false;
  }
#endif

  current_ = true;
  enabled_ = true;

  RCLCPP_INFO(node->get_logger(),
              "DepthObstacleLayer: initialized (stride=%d, range=%.1fm, "
              "height=[%.2f,%.2f], "
              "human_mask=%s)",
              stride_, obstacle_range_, min_obstacle_height_,
              max_obstacle_height_,
              human_mask_enabled_ ? "enabled" : "disabled");
}

void DepthObstacleLayer::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  cam_model_.fromCameraInfo(msg);
  camera_info_received_ = true;

  RCLCPP_DEBUG_ONCE(rclcpp::get_logger("nav2_depth_obstacle_layer"),
                    "Camera info received, frame='%s', resolution=%dx%d",
                    cam_model_.tfFrame().c_str(), msg->width, msg->height);
}

void DepthObstacleLayer::depthCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  // Use toCvShare to avoid copy when encoding matches
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr =
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("nav2_depth_obstacle_layer"),
                          *clock_, 5000, "cv_bridge conversion failed: %s",
                          e.what());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    // Swap pre-allocated buffer to avoid allocation
    if (depth_buffer_.empty() || depth_buffer_.rows != cv_ptr->image.rows ||
        depth_buffer_.cols != cv_ptr->image.cols) {
      depth_buffer_ = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    }
    cv_ptr->image.copyTo(depth_buffer_);
    std::swap(depth_image_, depth_buffer_);
    last_depth_stamp_ = msg->header.stamp;
  }
}

#ifdef HAVE_ZED_MSGS
void DepthObstacleLayer::humanCallback(
    const zed_msgs::msg::ObjectsStamped::SharedPtr msg) {
  auto node = node_.lock();
  if (!node) {
    return;
  }

  const rclcpp::Time now = node->now();

  std::lock_guard<std::mutex> lock(human_mutex_);
  human_bboxes_.clear();
  human_bboxes_.reserve(msg->objects.size());

  for (const auto &obj : msg->objects) {
    // ZED bounding_box_2d.corners: [top-left, top-right, bottom-right,
    // bottom-left]
    if (obj.bounding_box_2d.corners.size() < 4) {
      RCLCPP_DEBUG(
          node->get_logger(),
          "Skipping object with invalid bounding_box_2d corners size: %zu",
          obj.bounding_box_2d.corners.size());
      continue;
    }

    ImageBBox bbox;
    bbox.u_min = static_cast<int>(obj.bounding_box_2d.corners[0].kp[0]) -
                 human_mask_padding_;
    bbox.v_min = static_cast<int>(obj.bounding_box_2d.corners[0].kp[1]) -
                 human_mask_padding_;
    bbox.u_max = static_cast<int>(obj.bounding_box_2d.corners[2].kp[0]) +
                 human_mask_padding_;
    bbox.v_max = static_cast<int>(obj.bounding_box_2d.corners[2].kp[1]) +
                 human_mask_padding_;
    bbox.stamp = now;

    // Clamp to non-negative (upper bound checked during pixel iteration)
    bbox.u_min = std::max(0, bbox.u_min);
    bbox.v_min = std::max(0, bbox.v_min);

    human_bboxes_.push_back(bbox);
  }

  RCLCPP_DEBUG(node->get_logger(),
               "Human detection: %zu objects received, %zu valid bboxes",
               msg->objects.size(), human_bboxes_.size());
}
#endif

void DepthObstacleLayer::removeStaleHumans() {
  auto node = node_.lock();
  if (!node) {
    return;
  }

  const rclcpp::Time now = node->now();
  std::lock_guard<std::mutex> lock(human_mutex_);

  const size_t before = human_bboxes_.size();
  human_bboxes_.erase(std::remove_if(human_bboxes_.begin(), human_bboxes_.end(),
                                     [&](const ImageBBox &b) {
                                       return (now - b.stamp).seconds() >
                                              human_persistence_;
                                     }),
                      human_bboxes_.end());

  if (human_bboxes_.size() != before) {
    RCLCPP_DEBUG(node->get_logger(),
                 "Removed %zu stale human detections, %zu remaining",
                 before - human_bboxes_.size(), human_bboxes_.size());
  }
}

void DepthObstacleLayer::updateBounds(double robot_x, double robot_y,
                                      double /*robot_yaw*/, double *min_x,
                                      double *min_y, double *max_x,
                                      double *max_y) {
  if (!enabled_) {
    return;
  }

  *min_x = std::min(*min_x, robot_x - obstacle_range_);
  *min_y = std::min(*min_y, robot_y - obstacle_range_);
  *max_x = std::max(*max_x, robot_x + obstacle_range_);
  *max_y = std::max(*max_y, robot_y + obstacle_range_);
}

void DepthObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                     int min_i, int min_j, int max_i,
                                     int max_j) {
  if (!enabled_) {
    return;
  }

  // Remove stale human detections
  if (human_mask_enabled_) {
    removeStaleHumans();
  }

  // Get depth image with minimal lock time
  cv::Mat depth_local;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (depth_image_.empty() || !camera_info_received_) {
      RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("nav2_depth_obstacle_layer"),
                            *clock_, 2000,
                            "Waiting for depth image and camera info");
      return;
    }
    depth_local = depth_image_; // Shallow copy (shared data)
  }

  // Copy human bboxes for thread-safe access
  std::vector<ImageBBox> bboxes_local;
  if (human_mask_enabled_) {
    std::lock_guard<std::mutex> lock(human_mutex_);
    bboxes_local = human_bboxes_;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::string camera_frame = cam_model_.tfFrame();
  if (camera_frame.empty()) {
    RCLCPP_WARN_ONCE(
        node->get_logger(),
        "Camera frame empty, using default 'camera_depth_optical_frame'");
    camera_frame = "camera_depth_optical_frame";
  }

  // TF lookups - both use TimePointZero for latest available
  geometry_msgs::msg::TransformStamped tf_cam_to_ground;
  geometry_msgs::msg::TransformStamped tf_cam_to_map;

  try {
    tf_cam_to_ground = tf_buffer_->lookupTransform(ground_frame_, camera_frame,
                                                   tf2::TimePointZero);
    tf_cam_to_map = tf_buffer_->lookupTransform(global_frame_, camera_frame,
                                                tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                         "TF lookup failed: %s", ex.what());
    return;
  }

  tf2::Transform T_cam_to_ground, T_cam_to_map;
  tf2::fromMsg(tf_cam_to_ground.transform, T_cam_to_ground);
  tf2::fromMsg(tf_cam_to_map.transform, T_cam_to_map);

  // Clamp bounds to grid size
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();
  min_i = std::clamp(min_i, 0, static_cast<int>(size_x));
  min_j = std::clamp(min_j, 0, static_cast<int>(size_y));
  max_i = std::clamp(max_i, 0, static_cast<int>(size_x));
  max_j = std::clamp(max_j, 0, static_cast<int>(size_y));

  // Clear update window using memset for speed
  if (max_i > min_i && max_j > min_j) {
    unsigned char *master_array = master_grid.getCharMap();
    const unsigned int width = size_x;
    for (int j = min_j; j < max_j; ++j) {
      std::memset(&master_array[j * width + min_i], nav2_costmap_2d::FREE_SPACE,
                  (max_i - min_i) * sizeof(unsigned char));
    }
  }

  // Cache camera intrinsics
  const float cx = cam_model_.cx();
  const float cy = cam_model_.cy();
  const float fx_inv = 1.0f / cam_model_.fx();
  const float fy_inv = 1.0f / cam_model_.fy();
  const float range_sq = static_cast<float>(obstacle_range_sq_);

  // Direct pointer access to depth data (avoid .at<> bounds checking)
  const int rows = depth_local.rows;
  const int cols = depth_local.cols;
  const float *depth_ptr = reinterpret_cast<const float *>(depth_local.data);
  const size_t step = depth_local.step1(); // Elements per row

  // Process depth image
  for (int v = 0; v < rows; v += stride_) {
    const float *row_ptr = depth_ptr + v * step;

    for (int u = 0; u < cols; u += stride_) {
      // Skip human regions (check before expensive depth processing)
      if (human_mask_enabled_) {
        bool in_human = false;
        for (const auto &bbox : bboxes_local) {
          if (u >= bbox.u_min && u <= bbox.u_max && v >= bbox.v_min &&
              v <= bbox.v_max) {
            in_human = true;
            break;
          }
        }
        if (in_human) {
          continue;
        }
      }

      const float d = row_ptr[u];

      // Validate depth - use squared comparison to avoid sqrt
      if (!std::isfinite(d) || d <= 0.0f || d * d > range_sq) {
        continue;
      }

      // Project to 3D in camera frame
      const float cam_x = (u - cx) * d * fx_inv;
      const float cam_y = (v - cy) * d * fy_inv;
      const float cam_z = d;

      const tf2::Vector3 point_cam(cam_x, cam_y, cam_z);

      // Height filtering in ground frame
      const tf2::Vector3 point_ground = T_cam_to_ground * point_cam;
      const double height = point_ground.z();

      if (height < min_obstacle_height_ || height > max_obstacle_height_) {
        continue;
      }

      // Project to costmap global frame
      const tf2::Vector3 point_map = T_cam_to_map * point_cam;

      unsigned int mx, my;
      if (master_grid.worldToMap(point_map.x(), point_map.y(), mx, my)) {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  RCLCPP_DEBUG_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                        "updateCosts complete, human_bboxes=%zu",
                        bboxes_local.size());
}

void DepthObstacleLayer::reset() {
  {
    std::lock_guard<std::mutex> lock(human_mutex_);
    human_bboxes_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth_image_.release();
    depth_buffer_.release();
  }

  RCLCPP_DEBUG(rclcpp::get_logger("nav2_depth_obstacle_layer"),
               "Layer reset complete");
}

bool DepthObstacleLayer::isClearable() { return true; }

} // namespace nav2_depth_obstacle_layer

PLUGINLIB_EXPORT_CLASS(nav2_depth_obstacle_layer::DepthObstacleLayer,
                       nav2_costmap_2d::Layer)
