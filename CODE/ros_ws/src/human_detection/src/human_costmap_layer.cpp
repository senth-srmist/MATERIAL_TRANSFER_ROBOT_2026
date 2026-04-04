#include "human_detection/human_costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace human_detection
{

HumanCostmapLayer::HumanCostmapLayer() {}

HumanCostmapLayer::~HumanCostmapLayer() {}

void HumanCostmapLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("human_topic", "/zed/obj_det/objects");
  declareParameter("inflation_radius", 0.8);
  declareParameter("persistence_time", 1.0);
  declareParameter("human_cost", 254);

  getParameter("human_topic", human_topic_);
  getParameter("inflation_radius", inflation_radius_);
  getParameter("persistence_time", persistence_time_);
  getParameter("human_cost", human_cost_);

  tf_buffer_ = layered_costmap_->getTfBuffer();

  sub_ = node->create_subscription<zed_interfaces::msg::ObjectsStamped>(
    human_topic_, 10,
    std::bind(&HumanCostmapLayer::humanCallback, this, std::placeholders::_1));

  current_ = true;

  RCLCPP_INFO(node->get_logger(), "HumanCostmapLayer initialized");
}

void HumanCostmapLayer::humanCallback(
  const zed_interfaces::msg::ObjectsStamped::SharedPtr msg)
{
  for (const auto & obj : msg->objects)
  {
    if (obj.label != "Person") continue;

    double x = obj.position[0];
    double y = obj.position[1];

    if (!transformToGlobalFrame(x, y, msg->header.frame_id))
      continue;

    Human h;
    h.x = x;
    h.y = y;
    h.stamp = msg->header.stamp;

    humans_.push_back(h);
  }
}

bool HumanCostmapLayer::transformToGlobalFrame(
  double & x, double & y,
  const std::string & source_frame)
{
  geometry_msgs::msg::PointStamped pt, pt_out;

  pt.header.frame_id = source_frame;
  pt.point.x = x;
  pt.point.y = y;
  pt.point.z = 0.0;

  try
  {
    tf_buffer_->transform(pt, pt_out, layered_costmap_->getGlobalFrameID());
    x = pt_out.point.x;
    y = pt_out.point.y;
    return true;
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_WARN(rclcpp::get_logger("HumanLayer"), "TF failed: %s", ex.what());
    return false;
  }
}

void HumanCostmapLayer::removeStaleHumans()
{
  auto node = node_.lock();
  rclcpp::Time now = node->now();

  humans_.erase(
    std::remove_if(humans_.begin(), humans_.end(),
      [&](const Human & h)
      {
        return (now - h.stamp).seconds() > persistence_time_;
      }),
    humans_.end());
}

void HumanCostmapLayer::updateBounds(
  double, double, double,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  removeStaleHumans();

  for (const auto & h : humans_)
  {
    *min_x = std::min(*min_x, h.x - inflation_radius_);
    *min_y = std::min(*min_y, h.y - inflation_radius_);
    *max_x = std::max(*max_x, h.x + inflation_radius_);
    *max_y = std::max(*max_y, h.y + inflation_radius_);
  }
}

void HumanCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j,
  int max_i, int max_j)
{
  unsigned int mx, my;

  for (const auto & h : humans_)
  {
    if (!master_grid.worldToMap(h.x, h.y, mx, my))
      continue;

    int radius_cells = inflation_radius_ / master_grid.getResolution();

    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
    {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        int nx = mx + dx;
        int ny = my + dy;

        if (nx < min_i || nx >= max_i || ny < min_j || ny >= max_j)
          continue;

        master_grid.setCost(nx, ny, human_cost_);
      }
    }
  }
}

void HumanCostmapLayer::reset()
{
  humans_.clear();
}

}  // namespace human_detection

PLUGINLIB_EXPORT_CLASS(
  human_detection::HumanCostmapLayer,
  nav2_costmap_2d::Layer)