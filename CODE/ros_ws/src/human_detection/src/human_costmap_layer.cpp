#include "human_detection/human_costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace human_detection {

HumanCostmapLayer::HumanCostmapLayer() {}

HumanCostmapLayer::~HumanCostmapLayer() {}

void HumanCostmapLayer::onInitialize() {
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  // Declare parameters with rclcpp::ParameterValue
  declareParameter("human_topic",
                   rclcpp::ParameterValue("/zed/zed_node/obj_det/objects"));
  declareParameter("inflation_radius", rclcpp::ParameterValue(0.8));
  declareParameter("persistence_time", rclcpp::ParameterValue(1.0));
  declareParameter("human_cost", rclcpp::ParameterValue(254));

  // Get parameters
  node->get_parameter(name_ + ".human_topic", human_topic_);
  node->get_parameter(name_ + ".inflation_radius", inflation_radius_);
  node->get_parameter(name_ + ".persistence_time", persistence_time_);
  node->get_parameter(name_ + ".human_cost", human_cost_);

  // Get TF buffer from the node
  tf_ = tf_; // tf_ is inherited from Layer base class

  sub_ = node->create_subscription<zed_msgs::msg::ObjectsStamped>(
      human_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HumanCostmapLayer::humanCallback, this,
                std::placeholders::_1));

  current_ = true;
  enabled_ = true;

  RCLCPP_INFO(node->get_logger(),
              "HumanCostmapLayer initialized: topic=%s, inflation=%.2f, "
              "persistence=%.2f",
              human_topic_.c_str(), inflation_radius_, persistence_time_);
}

void HumanCostmapLayer::humanCallback(
    const zed_msgs::msg::ObjectsStamped::SharedPtr msg) {
  auto node = node_.lock();
  if (!node)
    return;

  for (const auto &obj : msg->objects) {
    // ZED optical frame convention:
    //   X = right
    //   Y = down
    //   Z = forward (depth into the scene)
    //
    // All three components are required for correct 3D->2D transformation
    // to the global costmap frame
    double x = obj.position[0];
    double y = obj.position[1];
    double z = obj.position[2]; // Depth - critical for correct positioning!

    // Use the message timestamp for transform lookup to ensure temporal
    // consistency
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
  pt.header.stamp = stamp; // Use actual detection timestamp, not Time(0)
  pt.point.x = x;
  pt.point.y = y;
  pt.point.z = z; // Include depth for proper 3D transform

  try {
    tf_->transform(pt, pt_out, layered_costmap_->getGlobalFrameID(),
                   tf2::durationFromSec(0.5));
    x = pt_out.point.x;
    y = pt_out.point.y;
    // z not needed for 2D costmap after transform
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

  for (const auto &h : humans_) {
    *min_x = std::min(*min_x, h.x - inflation_radius_);
    *min_y = std::min(*min_y, h.y - inflation_radius_);
    *max_x = std::max(*max_x, h.x + inflation_radius_);
    *max_y = std::max(*max_y, h.y + inflation_radius_);
  }
}

void HumanCostmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                    int min_i, int min_j, int max_i,
                                    int max_j) {
  if (!enabled_)
    return;

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

        // Check if within circular radius
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
  current_ = true;
}

bool HumanCostmapLayer::isClearable() { return true; }

} // namespace human_detection

PLUGINLIB_EXPORT_CLASS(human_detection::HumanCostmapLayer,
                       nav2_costmap_2d::Layer)
