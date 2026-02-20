/**
 * @file tile_check_action.hpp
 * @brief Nav2 Behavior Tree action node for tile zone checking and map switching
 * 
 * This BT node:
 * 1. Gets robot pose from TF
 * 2. Checks trigger zones from tiles_config.yaml (parsed from 'connections')
 * 3. Calls /map_server/load_map if switch needed
 * 4. Clears costmap after switch
 * 
 * Memory overhead: ~1 MB (part of BT Navigator process)
 * CPU overhead: Only when ticked by BT
 */

#ifndef TILE_MANAGER__TILE_CHECK_ACTION_HPP_
#define TILE_MANAGER__TILE_CHECK_ACTION_HPP_

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace tile_manager
{

struct TriggerZone
{
  double x_min, x_max, y_min, y_max;
  int from_tile, to_tile;
  std::string heading;  // "+x", "-x", "+y", "-y"
};

class TileCheckAction : public BT::SyncActionNode
{
public:
  TileCheckAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config),
    current_tile_(1),
    last_switch_time_(std::chrono::steady_clock::now()),
    initialized_(false)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("config_file", "", "Path to tiles_config.yaml"),
    };
  }

  BT::NodeStatus tick() override
  {
    // Lazy initialization on first tick
    if (!initialized_) {
      if (!initialize()) {
        return BT::NodeStatus::FAILURE;
      }
      initialized_ = true;
    }

    // Get robot pose
    double x, y, yaw;
    if (!getRobotPose(x, y, yaw)) {
      return BT::NodeStatus::SUCCESS;  // No pose yet, continue navigation
    }

    // Cooldown check
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_switch_time_).count();
    if (elapsed < switch_cooldown_) {
      return BT::NodeStatus::SUCCESS;
    }

    // Check trigger zones
    std::string heading = yawToHeading(yaw);
    int target_tile = checkTriggerZone(x, y, heading);

    if (target_tile > 0 && target_tile != current_tile_) {
      RCLCPP_INFO(node_->get_logger(), 
        "Trigger zone hit at (%.2f, %.2f) heading %s -> tile %d",
        x, y, heading.c_str(), target_tile);
      
      if (switchTile(target_tile)) {
        current_tile_ = target_tile;
        last_switch_time_ = now;
        
        // Post-switch delay for costmap rebuild
        if (post_switch_delay_ > 0) {
          rclcpp::sleep_for(std::chrono::milliseconds(
            static_cast<int>(post_switch_delay_ * 1000)));
        }
      }
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  // State
  int current_tile_;
  std::chrono::steady_clock::time_point last_switch_time_;
  double switch_cooldown_{0.5};
  double post_switch_delay_{0.3};
  bool initialized_;

  // Config
  std::map<int, std::string> tiles_;
  std::vector<TriggerZone> trigger_zones_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_loader_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr costmap_clear_;

  bool initialize()
  {
    // Get node from blackboard
    if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
      std::cerr << "TileCheckAction: No ROS node in blackboard" << std::endl;
      return false;
    }

    // Load config
    std::string config_file;
    getInput("config_file", config_file);
    
    if (config_file.empty()) {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("tile_manager");
      config_file = pkg_share + "/config/tiles_config.yaml";
    }

    if (!loadConfig(config_file)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load config: %s", config_file.c_str());
      return false;
    }

    // Setup TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup service clients
    map_loader_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    costmap_clear_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entire_costmap");

    RCLCPP_INFO(node_->get_logger(), 
      "TileCheckAction initialized: %zu tiles, %zu zones, starting tile %d",
      tiles_.size(), trigger_zones_.size(), current_tile_);

    return true;
  }

  bool loadConfig(const std::string & path)
  {
    try {
      YAML::Node config = YAML::LoadFile(path);
      
      std::string pkg_share = ament_index_cpp::get_package_share_directory("tile_manager");
      std::string maps_dir = pkg_share + "/maps/";

      // ========================================
      // Parse tiles
      // ========================================
      if (config["tiles"]) {
        for (const auto & tile : config["tiles"]) {
          int id = tile.first.as<int>();
          std::string file = tile.second["file"].as<std::string>();
          tiles_[id] = maps_dir + file;
        }
      }

      // ========================================
      // Parse connections (NEW FORMAT)
      // ========================================
      // connections:
      //   1-2:
      //     overlap: [x_min, x_max, y_min, y_max]
      //     heading: "x"
      //
      // This creates TWO trigger zones per connection:
      //   - from_tile=1, to_tile=2, heading="+x"
      //   - from_tile=2, to_tile=1, heading="-x"
      
      if (config["connections"]) {
        for (const auto & conn : config["connections"]) {
          std::string key = conn.first.as<std::string>();  // e.g., "1-2"
          auto data = conn.second;
          
          // Parse tile IDs from key "1-2"
          int tile_a, tile_b;
          if (!parseConnectionKey(key, tile_a, tile_b)) {
            RCLCPP_WARN(node_->get_logger(), "Invalid connection key: %s", key.c_str());
            continue;
          }
          
          // Parse overlap bounds
          auto overlap = data["overlap"];
          double x_min = overlap[0].as<double>();
          double x_max = overlap[1].as<double>();
          double y_min = overlap[2].as<double>();
          double y_max = overlap[3].as<double>();
          
          // Parse heading axis (default: "x")
          std::string heading_axis = data["heading"].as<std::string>("x");
          
          // Create trigger zone: tile_a -> tile_b
          TriggerZone tz_forward;
          tz_forward.x_min = x_min;
          tz_forward.x_max = x_max;
          tz_forward.y_min = y_min;
          tz_forward.y_max = y_max;
          tz_forward.from_tile = tile_a;
          tz_forward.to_tile = tile_b;
          tz_forward.heading = "+" + heading_axis;  // "+x" or "+y"
          trigger_zones_.push_back(tz_forward);
          
          // Create trigger zone: tile_b -> tile_a
          TriggerZone tz_reverse;
          tz_reverse.x_min = x_min;
          tz_reverse.x_max = x_max;
          tz_reverse.y_min = y_min;
          tz_reverse.y_max = y_max;
          tz_reverse.from_tile = tile_b;
          tz_reverse.to_tile = tile_a;
          tz_reverse.heading = "-" + heading_axis;  // "-x" or "-y"
          trigger_zones_.push_back(tz_reverse);
        }
      }
      
      // ========================================
      // Parse settings
      // ========================================
      if (config["settings"]) {
        current_tile_ = config["settings"]["initial_tile"].as<int>(1);
        switch_cooldown_ = config["settings"]["switch_cooldown"].as<double>(0.5);
        post_switch_delay_ = config["settings"]["post_switch_delay"].as<double>(0.3);
      }

      return true;
    } catch (const std::exception & e) {
      std::cerr << "Config parse error: " << e.what() << std::endl;
      return false;
    }
  }

  /**
   * @brief Parse connection key like "1-2" into tile IDs
   * @param key Connection key string
   * @param tile_a First tile ID (output)
   * @param tile_b Second tile ID (output)
   * @return true if parsing succeeded
   */
  bool parseConnectionKey(const std::string & key, int & tile_a, int & tile_b)
  {
    size_t dash_pos = key.find('-');
    if (dash_pos == std::string::npos) {
      return false;
    }
    
    try {
      tile_a = std::stoi(key.substr(0, dash_pos));
      tile_b = std::stoi(key.substr(dash_pos + 1));
      return true;
    } catch (...) {
      return false;
    }
  }

  bool getRobotPose(double & x, double & y, double & yaw)
  {
    try {
      auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;

      // Extract yaw from quaternion
      auto & q = transform.transform.rotation;
      double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw = std::atan2(siny_cosp, cosy_cosp);

      return true;
    } catch (const tf2::TransformException & e) {
      return false;
    }
  }

  std::string yawToHeading(double yaw)
  {
    // Normalize to [-pi, pi]
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));

    // ±45° tolerance for each direction
    if (yaw >= -0.785 && yaw < 0.785) {
      return "+x";
    } else if (yaw >= 0.785 && yaw < 2.356) {
      return "+y";
    } else if (yaw >= -2.356 && yaw < -0.785) {
      return "-y";
    } else {
      return "-x";
    }
  }

  int checkTriggerZone(double x, double y, const std::string & heading)
  {
    for (const auto & zone : trigger_zones_) {
      if (zone.from_tile != current_tile_) {
        continue;
      }

      bool in_bounds = (x >= zone.x_min && x <= zone.x_max &&
                        y >= zone.y_min && y <= zone.y_max);
      bool heading_match = (zone.heading == heading);

      if (in_bounds && heading_match) {
        return zone.to_tile;
      }
    }
    return -1;
  }

  bool switchTile(int target_tile)
  {
    if (tiles_.find(target_tile) == tiles_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Unknown tile: %d", target_tile);
      return false;
    }

    // Wait for service
    if (!map_loader_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(node_->get_logger(), "map_server service unavailable");
      return false;
    }

    // Load map
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = tiles_[target_tile];

    auto start = std::chrono::steady_clock::now();
    auto future = map_loader_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "LoadMap service call failed");
      return false;
    }

    auto result = future.get();
    if (result->result != 0) {
      RCLCPP_ERROR(node_->get_logger(), "LoadMap failed with code: %d", result->result);
      return false;
    }

    auto elapsed = std::chrono::steady_clock::now() - start;
    double ms = std::chrono::duration<double, std::milli>(elapsed).count();

    // Clear costmap
    if (costmap_clear_->wait_for_service(std::chrono::seconds(1))) {
      auto clear_req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      costmap_clear_->async_send_request(clear_req);
    }

    RCLCPP_INFO(node_->get_logger(), "Tile switch: %d -> %d (%.1f ms)", 
      current_tile_, target_tile, ms);

    return true;
  }
};

}  // namespace tile_manager

#endif  // TILE_MANAGER__TILE_CHECK_ACTION_HPP_
