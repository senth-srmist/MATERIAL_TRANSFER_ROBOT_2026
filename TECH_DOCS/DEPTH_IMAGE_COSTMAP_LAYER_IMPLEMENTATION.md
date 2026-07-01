# DepthImageCostmapLayer Implementation Guide

## Background

Nav2's built-in `ObstacleLayer` in ROS2 Humble only supports `PointCloud2` and `LaserScan` — it does **not** support `DepthImage` directly. Our ZED camera produces a sparse point cloud even at maximum settings (VGA + COMPACT mode), but the depth map is dense and high quality.

Rather than using `depth_image_proc` to convert depth → PointCloud2 (adding ~50-100MB RAM and an extra node), we're building a custom Nav2 costmap layer plugin that directly consumes depth images.

### Why Custom Plugin?

| Approach | Memory | Latency | Control |
|----------|--------|---------|---------|
| `depth_image_proc` → ObstacleLayer | +50-100MB | Extra hop | None |
| **Custom DepthImageCostmapLayer** | Zero extra | Direct | Full (sampling, filtering) |

---

## Architecture

```
ZED Camera
    │
    ├── /zed/zed_node/depth/depth_registered (sensor_msgs/Image)
    │                           │
    │                           ▼
    │                   DepthImageCostmapLayer
    │                   ├── Subscribes to depth image
    │                   ├── Subscribes to obj_det/objects (human bboxes)
    │                   ├── MASKS OUT pixels inside human bounding boxes ◄── CRITICAL
    │                   ├── Samples remaining pixels at configurable stride
    │                   ├── Projects to 3D → transforms to map frame
    │                   └── Marks obstacles → Nav2 REPLANS AROUND them
    │
    ├── /zed/zed_node/depth/camera_info (sensor_msgs/CameraInfo)
    │
    └── /zed/zed_node/obj_det/objects (zed_msgs/ObjectsStamped)
                            │
                            ├──────────────────────────────────┐
                            │                                  │
                            ▼                                  ▼
                    HumanCostmapLayer                   DepthImageCostmapLayer
                    (marks humans)                      (uses bboxes to EXCLUDE humans)
                            │                                  │
                            ▼                                  ▼
                    BT: HumanBlockingPath              Nav2 Controller
                    → STOP + RAISE ALARM               → REPLAN + NAVIGATE
```

### Key Design: Separation of Concerns

| Obstacle Type | Detected By | Handled By | Nav2 Behavior |
|---------------|-------------|------------|---------------|
| **Humans** | ZED obj_det | `HumanCostmapLayer` + BT | **STOP + ALARM** |
| **Everything else** | Depth image (masked) | `DepthImageCostmapLayer` | **REPLAN + NAVIGATE** |

**Why mask humans in depth layer?**
- If both layers mark humans → Nav2 might try to replan around a human
- We want humans to trigger STOP, not replanning
- Masking ensures clean separation of behavior

---

## Package Location

Add to existing `human_detection` package (or create new `depth_costmap_layer` package):

```
ros_ws/src/human_detection/
├── include/human_detection/
│   ├── human_costmap_layer.hpp       # Existing
│   └── depth_image_costmap_layer.hpp # NEW
├── src/
│   ├── human_costmap_layer.cpp       # Existing
│   └── depth_image_costmap_layer.cpp # NEW
├── human_costmap_layer_plugin.xml    # UPDATE (add new plugin)
├── CMakeLists.txt                    # UPDATE
└── package.xml                       # UPDATE (add image_geometry)
```

---

## Dependencies

Add to `package.xml`:

```xml
<depend>image_geometry</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
```

Add to `CMakeLists.txt`:

```cmake
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# Add to existing library or create new one
add_library(depth_image_costmap_layer SHARED
  src/depth_image_costmap_layer.cpp
)

ament_target_dependencies(depth_image_costmap_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  image_geometry
  cv_bridge
)

pluginlib_export_plugin_description_file(nav2_costmap_2d depth_costmap_layer_plugin.xml)
```

---

## Header File

**`include/human_detection/depth_image_costmap_layer.hpp`**

```cpp
#ifndef DEPTH_IMAGE_COSTMAP_LAYER_HPP_
#define DEPTH_IMAGE_COSTMAP_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "zed_msgs/msg/objects_stamped.hpp"  // For human detection masking

#include <mutex>
#include <vector>
#include <string>

namespace human_detection
{

/**
 * @brief Nav2 costmap layer that directly consumes depth images
 * 
 * This plugin bypasses the need for PointCloud2 conversion by:
 * 1. Subscribing to depth image + camera_info
 * 2. Subscribing to ZED object detections to MASK OUT humans
 * 3. Sampling pixels at configurable stride (for performance)
 * 4. Skipping pixels inside human bounding boxes
 * 5. Projecting remaining pixels to 3D using camera intrinsics
 * 6. Transforming to global frame
 * 7. Marking/clearing costmap cells
 * 
 * IMPORTANT: Humans are handled by HumanCostmapLayer with STOP+ALARM behavior.
 * This layer handles all OTHER obstacles with AVOID+NAVIGATE behavior.
 * We must NOT mark humans here or Nav2 will try to replan around them.
 */
class DepthImageCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  DepthImageCostmapLayer();
  virtual ~DepthImageCostmapLayer();

  // Nav2 Layer interface
  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j,
    int max_i, int max_j) override;
  virtual void reset() override;
  virtual bool isClearable() override;

private:
  // Callbacks
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void objectsCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);

  // Processing
  void processDepthImage(const sensor_msgs::msg::Image::SharedPtr & depth_msg);
  bool transformToGlobalFrame(
    double & x, double & y, double & z,
    const std::string & source_frame,
    const rclcpp::Time & stamp);
  
  // Human masking
  bool isPixelInsideHumanBbox(int u, int v) const;

  // ROS subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr objects_sub_;

  // Camera model (from image_geometry)
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_model_initialized_;
  std::mutex camera_model_mutex_;

  // Human bounding boxes (2D pixel coordinates) - for masking
  struct HumanBbox
  {
    int u_min, u_max;  // Pixel columns
    int v_min, v_max;  // Pixel rows
  };
  std::vector<HumanBbox> human_bboxes_;
  std::mutex human_bboxes_mutex_;
  rclcpp::Time last_objects_time_;
  double human_bbox_persistence_;  // How long to keep human masks valid

  // Obstacle storage
  struct Obstacle
  {
    double x;  // Global frame
    double y;  // Global frame
    double z;  // Global frame (for height filtering)
  };
  std::vector<Obstacle> obstacles_;
  std::mutex obstacles_mutex_;

  // Parameters
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string objects_topic_;       // ZED object detections
  std::string sensor_frame_;
  int sample_stride_;               // Process every Nth pixel (performance vs density)
  double min_obstacle_height_;      // Relative to ground (filter floor)
  double max_obstacle_height_;      // Filter ceiling/high objects
  double min_range_;                // Minimum depth to consider
  double max_range_;                // Maximum depth to consider
  double obstacle_persistence_;     // How long obstacles persist (seconds)
  bool marking_;                    // Mark obstacles
  bool clearing_;                   // Clear free space via raytrace
  bool mask_humans_;                // Whether to mask out human detections
  int human_bbox_padding_;          // Extra pixels around human bbox (safety margin)

  // Timing
  rclcpp::Time last_depth_time_;

}  // namespace human_detection

#endif  // DEPTH_IMAGE_COSTMAP_LAYER_HPP_
```

---

## Source File

**`src/depth_image_costmap_layer.cpp`**

```cpp
#include "human_detection/depth_image_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <limits>

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::NO_INFORMATION;

namespace human_detection
{

DepthImageCostmapLayer::DepthImageCostmapLayer()
: camera_model_initialized_(false)
{
}

DepthImageCostmapLayer::~DepthImageCostmapLayer()
{
}

void DepthImageCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  // Declare parameters
  declareParameter("depth_topic", rclcpp::ParameterValue("/zed/zed_node/depth/depth_registered"));
  declareParameter("camera_info_topic", rclcpp::ParameterValue("/zed/zed_node/depth/camera_info"));
  declareParameter("objects_topic", rclcpp::ParameterValue("/zed/zed_node/obj_det/objects"));
  declareParameter("sensor_frame", rclcpp::ParameterValue("zed_left_camera_optical_frame"));
  declareParameter("sample_stride", rclcpp::ParameterValue(4));  // Every 4th pixel
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(0.1));
  declareParameter("max_obstacle_height", rclcpp::ParameterValue(1.5));
  declareParameter("min_range", rclcpp::ParameterValue(0.2));
  declareParameter("max_range", rclcpp::ParameterValue(3.0));
  declareParameter("obstacle_persistence", rclcpp::ParameterValue(0.5));
  declareParameter("marking", rclcpp::ParameterValue(true));
  declareParameter("clearing", rclcpp::ParameterValue(true));
  declareParameter("mask_humans", rclcpp::ParameterValue(true));  // IMPORTANT: Default ON
  declareParameter("human_bbox_padding", rclcpp::ParameterValue(20));  // 20 pixel margin
  declareParameter("human_bbox_persistence", rclcpp::ParameterValue(0.3));  // 300ms

  // Get parameters
  node->get_parameter(name_ + ".depth_topic", depth_topic_);
  node->get_parameter(name_ + ".camera_info_topic", camera_info_topic_);
  node->get_parameter(name_ + ".objects_topic", objects_topic_);
  node->get_parameter(name_ + ".sensor_frame", sensor_frame_);
  node->get_parameter(name_ + ".sample_stride", sample_stride_);
  node->get_parameter(name_ + ".min_obstacle_height", min_obstacle_height_);
  node->get_parameter(name_ + ".max_obstacle_height", max_obstacle_height_);
  node->get_parameter(name_ + ".min_range", min_range_);
  node->get_parameter(name_ + ".max_range", max_range_);
  node->get_parameter(name_ + ".obstacle_persistence", obstacle_persistence_);
  node->get_parameter(name_ + ".marking", marking_);
  node->get_parameter(name_ + ".clearing", clearing_);
  node->get_parameter(name_ + ".mask_humans", mask_humans_);
  node->get_parameter(name_ + ".human_bbox_padding", human_bbox_padding_);
  node->get_parameter(name_ + ".human_bbox_persistence", human_bbox_persistence_);

  // Clamp stride to reasonable range
  sample_stride_ = std::max(1, std::min(sample_stride_, 16));

  // Subscribe to camera info first (needed for projection)
  camera_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&DepthImageCostmapLayer::cameraInfoCallback, this, std::placeholders::_1));

  // Subscribe to depth image
  depth_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&DepthImageCostmapLayer::depthCallback, this, std::placeholders::_1));

  // Subscribe to ZED object detections (for human masking)
  if (mask_humans_) {
    objects_sub_ = node->create_subscription<zed_msgs::msg::ObjectsStamped>(
      objects_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DepthImageCostmapLayer::objectsCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node->get_logger(),
      "Human masking ENABLED: pixels inside human bounding boxes will be skipped");
  }

  current_ = true;
  enabled_ = true;

  RCLCPP_INFO(node->get_logger(),
    "DepthImageCostmapLayer initialized: depth=%s, stride=%d, range=[%.2f, %.2f], "
    "height=[%.2f, %.2f], mask_humans=%s",
    depth_topic_.c_str(), sample_stride_, min_range_, max_range_,
    min_obstacle_height_, max_obstacle_height_,
    mask_humans_ ? "true" : "false");
}

void DepthImageCostmapLayer::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(camera_model_mutex_);
  camera_model_.fromCameraInfo(msg);
  camera_model_initialized_ = true;
}

void DepthImageCostmapLayer::objectsCallback(
  const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) return;

  std::vector<HumanBbox> new_bboxes;

  for (const auto & obj : msg->objects) {
    // ZED SDK: Person class is typically label_id 0
    // Check your ZED config - mc_people should be enabled
    // The label_id for PERSON may vary, commonly 0 or check zed_msgs
    
    // Skip non-person objects
    // Note: Adjust this check based on your ZED SDK version
    // Some versions use string label, others use label_id
    if (obj.label_id != 0) {  // 0 = PERSON in ZED SDK
      continue;
    }

    // ZED provides 2D bounding box in image coordinates
    // bounding_box_2d is typically [4][2] = 4 corners, each with (u, v)
    // We need min/max of u and v
    
    if (obj.bounding_box_2d.corners.size() < 4) {
      continue;
    }

    int u_min = std::numeric_limits<int>::max();
    int u_max = std::numeric_limits<int>::min();
    int v_min = std::numeric_limits<int>::max();
    int v_max = std::numeric_limits<int>::min();

    for (const auto & corner : obj.bounding_box_2d.corners) {
      int u = static_cast<int>(corner.kp[0]);
      int v = static_cast<int>(corner.kp[1]);
      u_min = std::min(u_min, u);
      u_max = std::max(u_max, u);
      v_min = std::min(v_min, v);
      v_max = std::max(v_max, v);
    }

    // Add padding for safety margin
    u_min -= human_bbox_padding_;
    u_max += human_bbox_padding_;
    v_min -= human_bbox_padding_;
    v_max += human_bbox_padding_;

    new_bboxes.push_back({u_min, u_max, v_min, v_max});
  }

  // Update human bboxes (thread-safe)
  {
    std::lock_guard<std::mutex> lock(human_bboxes_mutex_);
    human_bboxes_ = std::move(new_bboxes);
    last_objects_time_ = msg->header.stamp;
  }

  if (!new_bboxes.empty()) {
    RCLCPP_DEBUG(node->get_logger(),
      "Updated %zu human bounding boxes for masking", new_bboxes.size());
  }
}

bool DepthImageCostmapLayer::isPixelInsideHumanBbox(int u, int v) const
{
  // Note: Called with human_bboxes_mutex_ already locked by caller
  for (const auto & bbox : human_bboxes_) {
    if (u >= bbox.u_min && u <= bbox.u_max &&
        v >= bbox.v_min && v <= bbox.v_max)
    {
      return true;
    }
  }
  return false;
}

void DepthImageCostmapLayer::depthCallback(
  const sensor_msgs::msg::Image::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(camera_model_mutex_);
    if (!camera_model_initialized_) {
      return;  // Wait for camera info
    }
  }

  processDepthImage(msg);
}

void DepthImageCostmapLayer::processDepthImage(
  const sensor_msgs::msg::Image::SharedPtr & depth_msg)
{
  auto node = node_.lock();
  if (!node) return;

  // Convert to OpenCV
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    // ZED publishes 32FC1 depth images (meters)
    cv_ptr = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const cv::Mat & depth_image = cv_ptr->image;
  const int width = depth_image.cols;
  const int height = depth_image.rows;

  std::vector<Obstacle> new_obstacles;
  new_obstacles.reserve((width / sample_stride_) * (height / sample_stride_));

  // Get camera model (thread-safe copy)
  image_geometry::PinholeCameraModel model;
  {
    std::lock_guard<std::mutex> lock(camera_model_mutex_);
    model = camera_model_;
  }

  // Get human bboxes (thread-safe copy) and check freshness
  std::vector<HumanBbox> current_human_bboxes;
  bool human_bboxes_valid = false;
  if (mask_humans_) {
    std::lock_guard<std::mutex> lock(human_bboxes_mutex_);
    double bbox_age = (node->now() - last_objects_time_).seconds();
    if (bbox_age < human_bbox_persistence_) {
      current_human_bboxes = human_bboxes_;
      human_bboxes_valid = true;
    }
  }

  int skipped_human_pixels = 0;  // For debug logging

  // Sample pixels at stride
  for (int v = 0; v < height; v += sample_stride_) {
    for (int u = 0; u < width; u += sample_stride_) {
      
      // ============================================================
      // CRITICAL: Skip pixels inside human bounding boxes
      // Humans are handled by HumanCostmapLayer (STOP + ALARM)
      // We only handle non-human obstacles (AVOID + NAVIGATE)
      // ============================================================
      if (mask_humans_ && human_bboxes_valid) {
        bool inside_human = false;
        for (const auto & bbox : current_human_bboxes) {
          if (u >= bbox.u_min && u <= bbox.u_max &&
              v >= bbox.v_min && v <= bbox.v_max)
          {
            inside_human = true;
            break;
          }
        }
        if (inside_human) {
          skipped_human_pixels++;
          continue;  // Skip this pixel - human region
        }
      }

      float depth = depth_image.at<float>(v, u);

      // Filter invalid/out-of-range depths
      if (!std::isfinite(depth) || depth < min_range_ || depth > max_range_) {
        continue;
      }

      // Project pixel to 3D ray using camera model
      // cv::Point2d(u, v) -> 3D unit ray
      cv::Point3d ray = model.projectPixelTo3dRay(cv::Point2d(u, v));

      // Scale ray by depth to get 3D point in camera optical frame
      // Note: In optical frame, Z is forward, X is right, Y is down
      double cam_x = ray.x * depth;
      double cam_y = ray.y * depth;
      double cam_z = ray.z * depth;  // This equals depth for optical frame

      // Transform to global frame
      double global_x = cam_x;
      double global_y = cam_y;
      double global_z = cam_z;

      if (!transformToGlobalFrame(global_x, global_y, global_z,
          depth_msg->header.frame_id, depth_msg->header.stamp))
      {
        continue;  // TF not available
      }

      // Height filtering (global_z is height above ground after transform)
      if (global_z < min_obstacle_height_ || global_z > max_obstacle_height_) {
        continue;
      }

      new_obstacles.push_back({global_x, global_y, global_z});
    }
  }

  if (skipped_human_pixels > 0) {
    RCLCPP_DEBUG(node->get_logger(),
      "Skipped %d pixels inside human bounding boxes", skipped_human_pixels);
  }

  // Update obstacle list (thread-safe)
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    obstacles_ = std::move(new_obstacles);
    last_depth_time_ = depth_msg->header.stamp;
  }
}

bool DepthImageCostmapLayer::transformToGlobalFrame(
  double & x, double & y, double & z,
  const std::string & source_frame,
  const rclcpp::Time & stamp)
{
  auto node = node_.lock();
  if (!node) return false;

  geometry_msgs::msg::PointStamped in_point, out_point;
  in_point.header.frame_id = source_frame;
  in_point.header.stamp = stamp;
  in_point.point.x = x;
  in_point.point.y = y;
  in_point.point.z = z;

  try {
    // Transform to global frame (typically "map" or "odom")
    out_point = tf_->transform(
      in_point,
      layered_costmap_->getGlobalFrameID(),
      tf2::durationFromSec(0.1));

    x = out_point.point.x;
    y = out_point.point.y;
    z = out_point.point.z;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      1000,
      "TF transform failed: %s", ex.what());
    return false;
  }
}

void DepthImageCostmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  // Check for stale data
  auto node = node_.lock();
  if (node) {
    double age = (node->now() - last_depth_time_).seconds();
    if (age > obstacle_persistence_) {
      obstacles_.clear();
      return;  // Data too old, don't expand bounds
    }
  }

  // Expand bounds to include all obstacles
  for (const auto & obs : obstacles_) {
    *min_x = std::min(*min_x, obs.x);
    *min_y = std::min(*min_y, obs.y);
    *max_x = std::max(*max_x, obs.x);
    *max_y = std::max(*max_y, obs.y);
  }
}

void DepthImageCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(obstacles_mutex_);

  // Mark obstacle cells
  if (marking_) {
    for (const auto & obs : obstacles_) {
      unsigned int mx, my;
      if (master_grid.worldToMap(obs.x, obs.y, mx, my)) {
        // Check bounds
        int ix = static_cast<int>(mx);
        int iy = static_cast<int>(my);
        if (ix >= min_i && ix < max_i && iy >= min_j && iy < max_j) {
          master_grid.setCost(mx, my, LETHAL_OBSTACLE);
        }
      }
    }
  }

  // Note: Clearing (raytracing) is more complex and optional
  // For now, we rely on the existing obstacle layer's clearing
  // or implement a simple clearing strategy if needed
}

void DepthImageCostmapLayer::reset()
{
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  obstacles_.clear();
  current_ = true;
}

bool DepthImageCostmapLayer::isClearable()
{
  return true;
}

}  // namespace human_detection

// Register the plugin
PLUGINLIB_EXPORT_CLASS(
  human_detection::DepthImageCostmapLayer,
  nav2_costmap_2d::Layer)
```

---

## Plugin XML

**`depth_costmap_layer_plugin.xml`** (or add to existing `human_costmap_layer_plugin.xml`):

```xml
<library path="depth_image_costmap_layer">
  <class name="human_detection/DepthImageCostmapLayer"
         type="human_detection::DepthImageCostmapLayer"
         base_class_type="nav2_costmap_2d::Layer">
    <description>
      Costmap layer that directly consumes depth images without PointCloud2 conversion.
      Samples depth pixels at configurable stride, projects to 3D, and marks obstacles.
    </description>
  </class>
</library>
```

---

## Nav2 Configuration

**Add to `nav2_params.yaml`:**

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["depth_layer", "human_layer", "inflation_layer"]
      
      depth_layer:
        plugin: "human_detection/DepthImageCostmapLayer"
        enabled: true
        depth_topic: "/zed/zed_node/depth/depth_registered"
        camera_info_topic: "/zed/zed_node/depth/camera_info"
        objects_topic: "/zed/zed_node/obj_det/objects"  # For human masking
        sensor_frame: "zed_left_camera_optical_frame"
        sample_stride: 4              # Every 4th pixel (VGA 672x376 → ~16k points)
        min_obstacle_height: 0.1      # 10cm above ground (filter floor)
        max_obstacle_height: 1.5      # 1.5m (filter ceiling)
        min_range: 0.2                # ZED-M minimum
        max_range: 3.0                # Effective range for indoor nav
        obstacle_persistence: 0.5     # Clear after 500ms
        marking: true
        clearing: false               # Let inflation handle clearing
        # Human masking config (CRITICAL for correct behavior)
        mask_humans: true             # Skip pixels inside human bboxes
        human_bbox_padding: 20        # Extra 20 pixel margin around humans
        human_bbox_persistence: 0.3   # Keep masks valid for 300ms

      human_layer:
        plugin: "human_detection/HumanCostmapLayer"
        # ... existing config ...
        # This layer marks humans -> triggers STOP + ALARM via BT

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        # ... existing config ...
```

### Behavior Summary

| Layer | Handles | Nav2 Behavior |
|-------|---------|---------------|
| `depth_layer` | Walls, furniture, objects (NOT humans) | Replan around |
| `human_layer` | Humans only | Stop + alarm (via BT) |
| `inflation_layer` | Buffer around all obstacles | Safety margin |
```

---

## Testing Checklist

### Phase 1: Build & Load

```bash
# Build
cd /workspace/ros_ws
colcon build --packages-select human_detection

# Source
source install/setup.bash

# Verify plugin is registered
ros2 pkg prefix human_detection
find $(ros2 pkg prefix human_detection) -name "*.so" | xargs -I{} nm -C {} | grep DepthImage
```

### Phase 2: Runtime Verification

```bash
# Launch robot with nav2
ros2 launch robot_bringup robot_bringup.launch.py

# Check if layer loaded
ros2 topic echo /local_costmap/costmap --once

# Monitor for errors
ros2 topic echo /rosout | grep -i depth
```

### Phase 3: Human Masking Verification (CRITICAL)

This phase ensures humans are NOT marked by the depth layer.

```bash
# 1. Check human detections are coming in
ros2 topic echo /zed/zed_node/obj_det/objects --field objects

# 2. Monitor debug output (temporarily set log level to DEBUG)
ros2 param set /local_costmap/local_costmap depth_layer.log_level DEBUG

# 3. Look for "Skipped X pixels inside human bounding boxes" messages
ros2 topic echo /rosout | grep -i "Skipped.*human"
```

**Manual Test:**
1. Stand in front of robot (you should be detected as human)
2. Observe costmap in RViz:
   - `HumanCostmapLayer` should show obstacle at your position (red/lethal)
   - `DepthImageCostmapLayer` should have a "hole" where you are
3. Nav2 should trigger STOP + ALARM, NOT try to replan around you

**If both layers mark you:**
- Check `mask_humans: true` in config
- Verify `/zed/zed_node/obj_det/objects` is publishing person detections
- Check `human_bbox_persistence` isn't too short

### Phase 4: Visualization

```bash
# In RViz, add:
# - /local_costmap/costmap (Map display)
# - /zed/zed_node/depth/depth_registered (Image display)

# Compare depth image coverage vs costmap marks
```

### Phase 5: Performance Tuning

| Parameter | Effect | Tune For |
|-----------|--------|----------|
| `sample_stride` | ↑ = fewer points, faster | Jetson: try 4-8 |
| `obstacle_persistence` | ↑ = obstacles linger | Slow robot: 0.5-1.0s |
| `max_range` | ↑ = more points | Indoor: 3m is usually enough |
| `human_bbox_padding` | ↑ = larger masked area | Increase if human edges leak |
| `human_bbox_persistence` | ↑ = masks stay valid longer | Increase if detection drops frames |

---

## Stride Calculation

At VGA resolution (672×376) with stride 4:

```
Points per frame = (672/4) * (376/4) = 168 * 94 = ~15,792 points
```

Compare to ZED's COMPACT point cloud which gives ~2,000-5,000 points. This is 3-8x denser while still being efficient.

---

## Common Issues

### 1. "TF transform failed"

The depth image frame (`zed_left_camera_optical_frame`) must be in the TF tree. Ensure ZED is publishing TF or your URDF has the correct frames.

### 2. "cv_bridge exception"

ZED publishes 32FC1 depth. If you see this error, check the depth image encoding:

```bash
ros2 topic echo /zed/zed_node/depth/depth_registered --field encoding
```

### 3. Obstacles not appearing

- Check `min_obstacle_height` / `max_obstacle_height` — are they correct for your camera mount?
- Verify TF: is `zed_left_camera_optical_frame` → `map` working?
- Check depth image is publishing at expected rate

### 4. Performance issues

- Increase `sample_stride` (e.g., 4 → 8)
- Decrease `max_range` (e.g., 3.0 → 2.0)
- Run `htop` and monitor CPU usage

### 5. Humans being marked by depth layer (CRITICAL BUG)

**Symptom:** Nav2 tries to replan around humans instead of stopping.

**This breaks the intended behavior:** humans should trigger STOP + ALARM, not replanning.

**Debug checklist:**
```bash
# 1. Is mask_humans enabled?
ros2 param get /local_costmap/local_costmap depth_layer.mask_humans

# 2. Is object detection publishing?
ros2 topic hz /zed/zed_node/obj_det/objects

# 3. Are you detected as a person?
ros2 topic echo /zed/zed_node/obj_det/objects --field objects
# Look for label_id: 0 (PERSON)

# 4. Check debug logs
ros2 topic echo /rosout | grep -i "human bounding boxes"
```

**Fixes:**
- Set `mask_humans: true` in nav2_params.yaml
- Increase `human_bbox_persistence` to 0.5s if detections are dropping frames
- Increase `human_bbox_padding` to 30-40 if edges leak through
- Ensure ZED config has `mc_people: true`

### 6. Human walks through without robot stopping

This is a `HumanCostmapLayer` or BT issue, not `DepthImageCostmapLayer`:
- Check ZED object detection: `mc_people: true` in zedm.yaml
- Verify `HumanBlockingPath` BT node is in your behavior tree
- Check human detection confidence threshold

---

## Future Improvements

1. **Clearing via raytracing**: Implement proper ray clearing to mark free space
2. **Confidence filtering**: Use ZED confidence map to reject low-confidence depths
3. **Temporal filtering**: Average multiple frames to reduce noise
4. **Region of interest**: Only process center portion of image (ignore edges)

---

## References

- [Nav2 Costmap2D Plugins](https://navigation.ros.org/plugins/index.html)
- [image_geometry::PinholeCameraModel](http://docs.ros.org/en/noetic/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html)
- [HumanCostmapLayer implementation](../human_costmap_layer.cpp) — reference for plugin structure
