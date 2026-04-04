#ifndef HUMAN_COSTMAP_LAYER_HPP_
#define HUMAN_COSTMAP_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zed_msgs/msg/objects_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <vector>
#include <string>
#include <algorithm>

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

  virtual bool isClearable() override;

private:
  void humanCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg);

  bool transformToGlobalFrame(
    double & x, double & y,
    const std::string & source_frame);

  void removeStaleHumans();

  // ROS
  rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr sub_;

  // Parameters
  std::string human_topic_;
  double inflation_radius_;
  double persistence_time_;
  int human_cost_;

  // Data
  std::vector<Human> humans_;
};

}  // namespace human_detection

#endif  // HUMAN_COSTMAP_LAYER_HPP_
