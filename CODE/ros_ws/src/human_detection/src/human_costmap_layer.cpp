#include "human_detection/human_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::FREE_SPACE;

namespace human_detection {

HumanCostmapLayer::HumanCostmapLayer() {}

HumanCostmapLayer::~HumanCostmapLayer() {}

void HumanCostmapLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameter("human_topic",
                   rclcpp::ParameterValue("/zed/zed_node/obj_det/objects"));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.15));
  declareParameter("persistence_time", rclcpp::ParameterValue(1.0));
  declareParameter("human_cost", rclcpp::ParameterValue(128));

  node->get_parameter(name_ + ".human_topic", human_topic_);
  node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(name_ + ".persistence_time", persistence_time_);
  node->get_parameter(name_ + ".human_cost", human_cost_);

  sub_ = node->create_subscription<zed_msgs::msg::ObjectsStamped>(
      human_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HumanCostmapLayer::humanCallback, this,
                std::placeholders::_1));

  current_ = true;
  enabled_ = true;

  RCLCPP_INFO(node->get_logger(),
              "HumanCostmapLayer initialized: topic=%s, inflation=%.2f, "
              "persistence=%.2f, human_cost=%d",
              human_topic_.c_str(), inflation_radius_, persistence_time_,
              human_cost_);
}

void HumanCostmapLayer::humanCallback(
    const zed_msgs::msg::ObjectsStamped::SharedPtr msg) {
  auto node = node_.lock();
  if (!node)
    return;

  // Clear and rebuild on every message.
  // When ZED reports 0 objects the vector becomes empty immediately.
  // persistence_time_ is a fallback timeout if the topic goes silent.
  humans_.clear();

  for (const auto &obj : msg->objects) {
    double x = obj.position[0];
    double y = obj.position[1];
    double z = obj.position[2];

    if (!transformToGlobalFrame(x, y, z, msg->header.frame_id,
                                msg->header.stamp)) {
      continue;
    }

    Human h;
    h.x = x;
    h.y = y;
    h.stamp = node->now();
    humans_.push_back(h);
  }
}

bool HumanCostmapLayer::transformToGlobalFrame(double &x, double &y, double z,
                                               const std::string &source_frame,
                                               const rclcpp::Time &stamp) {
  if (!tf_) {
    return false;
  }

  geometry_msgs::msg::PointStamped pt, pt_out;
  pt.header.frame_id = source_frame;
  pt.header.stamp = stamp;
  pt.point.x = x;
  pt.point.y = y;
  pt.point.z = z;

  try {
    tf_->transform(pt, pt_out, layered_costmap_->getGlobalFrameID(),
                   tf2::durationFromSec(0.5));
    x = pt_out.point.x;
    y = pt_out.point.y;
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("HumanCostmapLayer"), *clock_, 1000,
                         "TF transform failed: %s", ex.what());
    return false;
  }
}

void HumanCostmapLayer::removeStaleHumans() {
  auto node = node_.lock();
  if (!node)
    return;

  rclcpp::Time now = node->now();
  humans_.erase(std::remove_if(humans_.begin(), humans_.end(),
                               [&](const Human &h) {
                                 return (now - h.stamp).seconds() >
                                        persistence_time_;
                               }),
                humans_.end());
}

void HumanCostmapLayer::updateBounds(double /*robot_x*/, double /*robot_y*/,
                                     double /*robot_yaw*/, double *min_x,
                                     double *min_y, double *max_x,
                                     double *max_y) {
  if (!enabled_)
    return;

  removeStaleHumans();

  // Compute this frame's bounds from current detections
  double cur_min_x = std::numeric_limits<double>::max();
  double cur_min_y = std::numeric_limits<double>::max();
  double cur_max_x = std::numeric_limits<double>::lowest();
  double cur_max_y = std::numeric_limits<double>::lowest();

  for (const auto &h : humans_) {
    cur_min_x = std::min(cur_min_x, h.x - inflation_radius_);
    cur_min_y = std::min(cur_min_y, h.y - inflation_radius_);
    cur_max_x = std::max(cur_max_x, h.x + inflation_radius_);
    cur_max_y = std::max(cur_max_y, h.y + inflation_radius_);
  }

  // FIX: Always include the PREVIOUS frame's bounds in the update window.
  //
  // Without this, when humans_ becomes empty (detection gone), updateBounds
  // expands nothing, so the costmap updater never revisits the cells we
  // painted last frame — they stay dirty at human_cost_ forever.
  //
  // By union-ing previous bounds here, updateCosts will be called over the
  // old painted region and will find no humans there, so it will write
  // FREE_SPACE, effectively clearing the stale cost.
  if (has_prev_bounds_) {
    cur_min_x = std::min(cur_min_x, prev_min_x_);
    cur_min_y = std::min(cur_min_y, prev_min_y_);
    cur_max_x = std::max(cur_max_x, prev_max_x_);
    cur_max_y = std::max(cur_max_y, prev_max_y_);
  }

  // Only publish valid (non-empty) bounds to the master costmap
  if (cur_min_x <= cur_max_x && cur_min_y <= cur_max_y) {
    *min_x = std::min(*min_x, cur_min_x);
    *min_y = std::min(*min_y, cur_min_y);
    *max_x = std::max(*max_x, cur_max_x);
    *max_y = std::max(*max_y, cur_max_y);

    // Save for next cycle
    prev_min_x_ = cur_min_x;
    prev_min_y_ = cur_min_y;
    prev_max_x_ = cur_max_x;
    prev_max_y_ = cur_max_y;
    has_prev_bounds_ = true;
  } else if (has_prev_bounds_) {
    // No current detections but we have a previous window — expand to cover it
    // so updateCosts clears it, then forget it.
    *min_x = std::min(*min_x, prev_min_x_);
    *min_y = std::min(*min_y, prev_min_y_);
    *max_x = std::max(*max_x, prev_max_x_);
    *max_y = std::max(*max_y, prev_max_y_);
    has_prev_bounds_ = false; // cleared; no need to revisit next cycle
  }
}

void HumanCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                    int min_i, int min_j, int max_i,
                                    int max_j) {
  if (!enabled_)
    return;

  // FIX: Clear the entire update window first.
  //
  // Previously the code only ever *set* costs, never reset them.
  // Cells painted in a prior cycle would persist in the master grid
  // indefinitely even after the human left.
  //
  // We reset every cell in the update window to FREE_SPACE, then repaint
  // only the cells that still have active detections. This is safe because:
  //   - The update window already includes the union of previous + current
  //     human bounding boxes (guaranteed by updateBounds above).
  //   - Other layers (static, inflation) will composite on top via their own
  //     updateCosts calls, so we are not clobbering their data — each layer
  //     owns only what it writes.
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      master_grid.setCost(i, j, FREE_SPACE);
    }
  }

  // Now repaint current detections
  unsigned int mx, my;
  for (const auto &h : humans_) {
    if (!master_grid.worldToMap(h.x, h.y, mx, my)) {
      continue;
    }

    int radius_cells =
        static_cast<int>(inflation_radius_ / master_grid.getResolution());

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        int nx = static_cast<int>(mx) + dx;
        int ny = static_cast<int>(my) + dy;

        if (nx < min_i || nx >= max_i || ny < min_j || ny >= max_j) {
          continue;
        }

        double dist = std::hypot(dx, dy) * master_grid.getResolution();
        if (dist <= inflation_radius_) {
          master_grid.setCost(nx, ny, static_cast<unsigned char>(human_cost_));
        }
      }
    }
  }
}

void HumanCostmapLayer::reset() {
  humans_.clear();
  has_prev_bounds_ = false;
  current_ = true;
}

bool HumanCostmapLayer::isClearable() { return true; }

} // namespace human_detection

PLUGINLIB_EXPORT_CLASS(human_detection::HumanCostmapLayer,
                       nav2_costmap_2d::Layer)
