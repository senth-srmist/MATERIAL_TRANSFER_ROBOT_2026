#include "depth_costmap_layer/depth_image_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace depth_costmap_layer {

DepthImageCostmapLayer::DepthImageCostmapLayer()
    : camera_info_received_(false),
      obstacle_range_(3.0) // default to prevent huge bounds
{}

void DepthImageCostmapLayer::onInitialize() {
  auto node = node_.lock();

  // Declare and get parameters
  node->declare_parameter("stride", 4);
  node->declare_parameter("min_obstacle_height", 0.05);
  node->declare_parameter("max_obstacle_height", 1.5);
  node->declare_parameter("obstacle_range", 3.0);
  node->declare_parameter("ground_frame", "ground");
  node->declare_parameter("height_frame", "camera_mount");

  stride_ = node->get_parameter("stride").as_int();
  min_obstacle_height_ = node->get_parameter("min_obstacle_height").as_double();
  max_obstacle_height_ = node->get_parameter("max_obstacle_height").as_double();
  obstacle_range_ = node->get_parameter("obstacle_range").as_double();
  ground_frame_ = node->get_parameter("ground_frame").as_string();
  height_frame_ = node->get_parameter("height_frame").as_string();

  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Subscribers
  depth_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
      "/zed/zed_node/depth/depth_registered", 10,
      std::bind(&DepthImageCostmapLayer::depthCallback, this,
                std::placeholders::_1));

  info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/zed/zed_node/depth/camera_info", 10,
      std::bind(&DepthImageCostmapLayer::cameraInfoCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(),
              "DepthImageCostmapLayer initialized. Stride: %d, Height range: "
              "[%.2f, %.2f], Range: %.2f m, Ground frame: %s, Height frame: %s",
              stride_, min_obstacle_height_, max_obstacle_height_,
              obstacle_range_, ground_frame_.c_str(), height_frame_.c_str());
}

void DepthImageCostmapLayer::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  cam_model_.fromCameraInfo(msg);
  camera_info_received_ = true;
  RCLCPP_INFO_ONCE(rclcpp::get_logger("depth_layer"),
                   "Camera info received. Frame: %s",
                   cam_model_.tfFrame().c_str());
}

void DepthImageCostmapLayer::depthCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(depth_mutex_);
  try {
    depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    last_depth_stamp_ = msg->header.stamp;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_layer"), "CV Bridge error: %s",
                 e.what());
  }
}

void DepthImageCostmapLayer::updateBounds(double robot_x, double robot_y,
                                          double, double *min_x, double *min_y,
                                          double *max_x, double *max_y) {
  *min_x = robot_x - obstacle_range_;
  *min_y = robot_y - obstacle_range_;
  *max_x = robot_x + obstacle_range_;
  *max_y = robot_y + obstacle_range_;
}

void DepthImageCostmapLayer::updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
    int max_j) {

  // Local copy of depth image to avoid holding mutex
  cv::Mat depth_copy;
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    if (depth_image_.empty() || !camera_info_received_) {
      return;
    }
    depth_image_.copyTo(depth_copy);
  }

  auto node = node_.lock();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_layer"), "Node pointer expired");
    return;
  }

  std::string camera_frame = cam_model_.tfFrame();
  if (camera_frame.empty()) {
    camera_frame = "zed_left_camera_optical_frame";
  }

  // 1. Transform for height: camera_optical -> ground (static chain, use
  // TimePointZero)
  geometry_msgs::msg::TransformStamped tf_cam_to_ground;
  try {
    tf_cam_to_ground = tf_buffer_->lookupTransform(ground_frame_, camera_frame,
                                                   tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                         "TF error (camera -> %s): %s", ground_frame_.c_str(),
                         ex.what());
    return;
  }

  // 2. Transform for XY: camera_optical -> map (dynamic, use latest available)
  geometry_msgs::msg::TransformStamped tf_cam_to_map;
  try {
    tf_cam_to_map = tf_buffer_->lookupTransform(global_frame_, camera_frame,
                                                tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                         "TF error (camera -> %s): %s", global_frame_.c_str(),
                         ex.what());
    return;
  }

  // Optional: fetch camera mount height for diagnostic (not used in height
  // calc)
  geometry_msgs::msg::TransformStamped tf_ground_to_height;
  double camera_height = 0.0;
  try {
    tf_ground_to_height = tf_buffer_->lookupTransform(
        ground_frame_, height_frame_, tf2::TimePointZero);
    camera_height = tf_ground_to_height.transform.translation.z;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                         "TF error (%s -> %s): %s", ground_frame_.c_str(),
                         height_frame_.c_str(), ex.what());
  }

  tf2::Transform T_cam_to_ground, T_cam_to_map;
  tf2::fromMsg(tf_cam_to_ground.transform, T_cam_to_ground);
  tf2::fromMsg(tf_cam_to_map.transform, T_cam_to_map);

  // Bounds checking for clearing loop
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  min_i = std::max(0, std::min(static_cast<int>(size_x), min_i));
  min_j = std::max(0, std::min(static_cast<int>(size_y), min_j));
  max_i = std::max(0, std::min(static_cast<int>(size_x), max_i));
  max_j = std::max(0, std::min(static_cast<int>(size_y), max_j));

  if (max_i > min_i && max_j > min_j) {
    unsigned char *master_array = master_grid.getCharMap();
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        master_array[master_grid.getIndex(i, j)] = nav2_costmap_2d::FREE_SPACE;
      }
    }
  }

  const unsigned char OBSTACLE_COST = nav2_costmap_2d::LETHAL_OBSTACLE;

  static bool logged_once = false;
  if (!logged_once) {
    RCLCPP_INFO(node->get_logger(), "Camera mount height from %s to %s: %.3f m",
                ground_frame_.c_str(), height_frame_.c_str(), camera_height);
  }

  for (int v = 0; v < depth_copy.rows; v += stride_) {
    for (int u = 0; u < depth_copy.cols; u += stride_) {
      float d = 0.0f;
      try {
        d = depth_copy.at<float>(v, u);
      } catch (const cv::Exception &e) {
        continue;
      }
      if (std::isnan(d) || std::isinf(d) || d <= 0.0f || d > obstacle_range_)
        continue;

      double cam_x = (u - cam_model_.cx()) * d / cam_model_.fx();
      double cam_y = (v - cam_model_.cy()) * d / cam_model_.fy();
      double cam_z = d;

      tf2::Vector3 point_cam(cam_x, cam_y, cam_z);
      tf2::Vector3 point_ground = T_cam_to_ground * point_cam;
      double height = point_ground.z(); // true height above floor

      if (!logged_once) {
        RCLCPP_INFO(
            node->get_logger(),
            "Sample: cam(%.2f,%.2f,%.2f) -> ground(%.2f,%.2f,%.2f) height=%.2f",
            cam_x, cam_y, cam_z, point_ground.x(), point_ground.y(),
            point_ground.z(), height);
        logged_once = true;
      }

      if (height < min_obstacle_height_ || height > max_obstacle_height_)
        continue;

      tf2::Vector3 point_map = T_cam_to_map * point_cam;
      unsigned int mx, my;
      if (master_grid.worldToMap(point_map.x(), point_map.y(), mx, my)) {
        master_grid.setCost(mx, my, OBSTACLE_COST);
      }
    }
  }
}

void DepthImageCostmapLayer::reset() {
  RCLCPP_INFO(rclcpp::get_logger("depth_layer"), "Depth layer reset");
}

bool DepthImageCostmapLayer::isClearable() { return true; }

} // namespace depth_costmap_layer

PLUGINLIB_EXPORT_CLASS(depth_costmap_layer::DepthImageCostmapLayer,
                       nav2_costmap_2d::Layer)
