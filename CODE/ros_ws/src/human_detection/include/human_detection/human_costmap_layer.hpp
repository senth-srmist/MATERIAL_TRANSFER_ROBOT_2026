#ifndef HUMAN_COSTMAP_LAYER_HPP_
#define HUMAN_COSTMAP_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_interfaces/msg/objects_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point.hpp"

#include <vector>

namespace human_detection
{

struct Human
{
  double x;
  double y;
  rclcpp::Time stamp;
};

class HumanCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  HumanCostmapLayer();
  virtual ~HumanCostmapLayer();

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

private:
  void humanCallback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg);

  bool transformToGlobalFrame(
    double & x, double & y,
    const std::string & source_frame);

  void removeStaleHumans();

  // ROS
  rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr sub_;
  tf2_ros::Buffer * tf_buffer_;

  // Parameters
  std::string human_topic_;
  double inflation_radius_;
  double persistence_time_;
  int human_cost_;

  // Data
  std::vector<Human> humans_;
};

}  // namespace human_detection

#endif