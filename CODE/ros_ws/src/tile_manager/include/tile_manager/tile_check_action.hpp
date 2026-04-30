/**
 * @file tile_check_action.hpp
 * @brief Nav2 Behavior Tree action node for tile zone checking and map
 * switching
 *
 * This BT node:
 * 1. Gets robot pose from TF
 * 2. Checks trigger zones from tiles_config.yaml (parsed from 'connections')
 * 3. Calls /map_server/load_map if switch needed (non-blocking)
 * 4. Clears costmap after switch (with result checking)
 *
 * Tile state persistence:
 * - Reads current tile from /tmp/current_tile.txt on init
 * - Writes to /tmp/current_tile.txt on every tile change
 * - No ROS topic used - file is the single source of truth
 *
 * Uses SyncActionNode (BT.CPP v3 compatible) with an internal state machine
 * to avoid blocking. Each tick() returns SUCCESS immediately — the async
 * service calls are polled across ticks via SwitchState.
 *
 * IMPORTANT: Because SyncActionNode cannot return RUNNING, the BT will NOT
 * pause FollowPath during a tile switch. The ReactiveSequence in the BT
 * relies on SUCCESS to keep ticking. This means the robot may briefly
 * drive on a stale costmap during the ~100-300ms switch window. This is
 * acceptable for our use case — the overlap zones are sized to absorb it.
 *
 * Memory overhead: ~1 MB (part of BT Navigator process)
 * CPU overhead: Only when ticked by BT
 */

#ifndef TILE_MANAGER__TILE_CHECK_ACTION_HPP_
#define TILE_MANAGER__TILE_CHECK_ACTION_HPP_

#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

namespace tile_manager {

// Persistent tile state file - single source of truth
constexpr const char* TILE_STATE_FILE = "/tmp/current_tile.txt";

struct TriggerZone {
  double x_min, x_max, y_min, y_max;
  int from_tile, to_tile;
  std::string heading;  // "+x", "-x", "+y", "-y"
};

/**
 * @brief Internal state machine for async tile switching.
 *
 * States:
 *   IDLE        – check pose & trigger zones each tick
 *   LOADING_MAP – waiting for LoadMap service response
 *   CLEARING    – waiting for ClearEntireCostmap response
 *   POST_DELAY  – timestamp-based wait for costmap rebuild
 */
enum class SwitchState {
  IDLE,
  LOADING_MAP,
  CLEARING,
  POST_DELAY,
};

class TileCheckAction : public BT::SyncActionNode {
public:
  TileCheckAction(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config),
        current_tile_(1),
        last_switch_time_(std::chrono::steady_clock::now()),
        initialized_(false),
        switch_state_(SwitchState::IDLE),
        pending_tile_(-1) {}

  ~TileCheckAction() override {
    // Clean up ROS resources to avoid leaked subscriptions on BT reload
    tf_listener_.reset();
    tf_buffer_.reset();
    map_loader_.reset();
    costmap_clear_.reset();
  }

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("config_file", "",
                                   "Path to tiles_config.yaml"),
    };
  }

  BT::NodeStatus tick() override {
    // Lazy initialization on first tick
    if (!initialized_) {
      if (!initialize()) {
        return BT::NodeStatus::FAILURE;
      }
      initialized_ = true;
    }

    // If we're mid-switch, poll the async state machine
    if (switch_state_ != SwitchState::IDLE) {
      pollSwitch();
      return BT::NodeStatus::SUCCESS;
    }

    // --- IDLE: normal zone checking ---

    // Get robot pose
    double x, y, yaw;
    if (!getRobotPose(x, y, yaw)) {
      return BT::NodeStatus::SUCCESS;  // No pose yet, let navigation continue
    }

    // Cooldown check
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - last_switch_time_).count();
    if (elapsed < switch_cooldown_) {
      return BT::NodeStatus::SUCCESS;
    }

    // Check trigger zones
    std::string heading = yawToHeading(yaw);
    int target_tile = checkTriggerZone(x, y, heading);

    if (target_tile > 0 && target_tile != current_tile_) {
      RCLCPP_INFO(node_->get_logger(),
                  "Trigger zone hit at (%.2f, %.2f) heading %s -> tile %d", x,
                  y, heading.c_str(), target_tile);
      beginSwitch(target_tile);
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  // ========================================================================
  // State
  // ========================================================================
  int current_tile_;
  std::chrono::steady_clock::time_point last_switch_time_;
  double switch_cooldown_{0.5};
  double post_switch_delay_{0.3};
  bool initialized_;

  // Async switch state machine
  SwitchState switch_state_;
  int pending_tile_;
  std::chrono::steady_clock::time_point switch_start_time_;
  std::chrono::steady_clock::time_point post_delay_start_;

  // Service futures (SharedFuture so they're copyable/resettable)
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture map_future_;
  int64_t map_request_id_{0};  // stored so we can cancel on timeout
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture costmap_future_;

  // ========================================================================
  // Config
  // ========================================================================
  std::map<int, std::string> tiles_;
  std::vector<TriggerZone> trigger_zones_;

  // ========================================================================
  // ROS
  // ========================================================================
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_loader_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr costmap_clear_;

  // ========================================================================
  // Tile state persistence (file-based)
  // ========================================================================

  /**
   * @brief Read current tile from persistent file.
   * @return Tile ID if file exists and valid, -1 otherwise.
   */
  int readPersistedTile() {
    std::ifstream f(TILE_STATE_FILE);
    if (!f.good()) {
      return -1;
    }

    int tile;
    f >> tile;

    if (f.fail() || tile < 1) {
      return -1;
    }

    // Validate tile exists in config
    if (tiles_.find(tile) == tiles_.end()) {
      RCLCPP_WARN(node_->get_logger(),
                  "Persisted tile %d not in config, ignoring", tile);
      return -1;
    }

    return tile;
  }

  /**
   * @brief Write current tile to persistent file (overwrites).
   */
  void persistTile(int tile_id) {
    std::ofstream f(TILE_STATE_FILE, std::ios::trunc);
    if (f.good()) {
      f << tile_id;
      f.close();
      RCLCPP_DEBUG(node_->get_logger(), "Persisted tile %d to %s",
                   tile_id, TILE_STATE_FILE);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to persist tile to %s",
                  TILE_STATE_FILE);
    }
  }

  // ========================================================================
  // Async tile switch state machine
  // ========================================================================

  /**
   * @brief Kick off an async tile switch.
   */
  void beginSwitch(int target_tile) {
    if (tiles_.find(target_tile) == tiles_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Unknown tile: %d", target_tile);
      return;
    }

    // Check service availability without blocking
    if (!map_loader_->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(),
                  "map_server service not ready, skipping switch");
      return;
    }

    // Send async LoadMap request; store request_id so we can cancel on timeout.
    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = tiles_[target_tile];

    auto fut_and_id = map_loader_->async_send_request(request);
    map_future_ = fut_and_id.future.share();
    map_request_id_ = fut_and_id.request_id;
    pending_tile_ = target_tile;
    switch_start_time_ = std::chrono::steady_clock::now();
    switch_state_ = SwitchState::LOADING_MAP;
  }

  /**
   * @brief Poll the async switch state machine. Non-blocking.
   *
   * Called every tick while switch_state_ != IDLE.
   * Always returns quickly — futures are polled with zero timeout.
   */
  void pollSwitch() {
    switch (switch_state_) {

    case SwitchState::LOADING_MAP: {
      // Check timeout (5 seconds)
      auto elapsed = std::chrono::steady_clock::now() - switch_start_time_;
      if (elapsed > std::chrono::seconds(5)) {
        RCLCPP_ERROR(node_->get_logger(), "LoadMap timed out after 5s");
        map_loader_->remove_pending_request(map_request_id_);
        resetSwitch();
        return;
      }

      // Poll future without blocking
      if (!map_future_.valid() ||
          map_future_.wait_for(std::chrono::milliseconds(0)) !=
              std::future_status::ready) {
        return;  // Still waiting, will poll again next tick
      }

      // Future is ready — check result
      try {
        auto result = map_future_.get();
        if (result->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
          RCLCPP_ERROR(node_->get_logger(), "LoadMap failed with code: %d",
                       result->result);
          resetSwitch();
          return;
        }

        RCLCPP_INFO(node_->get_logger(), "Map loaded for tile %d",
                    pending_tile_);

        // Update current tile and persist to file
        current_tile_ = pending_tile_;
        persistTile(current_tile_);

        // Start costmap clear
        if (costmap_clear_->service_is_ready()) {
          auto req =
              std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
          costmap_future_ =
              costmap_clear_->async_send_request(req).future.share();
          switch_state_ = SwitchState::CLEARING;
        } else {
          RCLCPP_WARN(node_->get_logger(),
                      "Costmap clear service not ready, skipping");
          post_delay_start_ = std::chrono::steady_clock::now();
          switch_state_ = SwitchState::POST_DELAY;
        }

      } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "LoadMap exception: %s", e.what());
        resetSwitch();
      }
      break;
    }

    case SwitchState::CLEARING: {
      auto elapsed = std::chrono::steady_clock::now() - switch_start_time_;
      if (elapsed > std::chrono::seconds(8)) {
        RCLCPP_WARN(node_->get_logger(), "Costmap clear timed out");
        post_delay_start_ = std::chrono::steady_clock::now();
        switch_state_ = SwitchState::POST_DELAY;
        return;
      }

      if (!costmap_future_.valid() ||
          costmap_future_.wait_for(std::chrono::milliseconds(0)) !=
              std::future_status::ready) {
        return;
      }

      RCLCPP_DEBUG(node_->get_logger(), "Costmap cleared");
      post_delay_start_ = std::chrono::steady_clock::now();
      switch_state_ = SwitchState::POST_DELAY;
      break;
    }

    case SwitchState::POST_DELAY: {
      auto elapsed = std::chrono::steady_clock::now() - post_delay_start_;
      if (elapsed >=
          std::chrono::duration<double>(post_switch_delay_)) {
        auto total_time = std::chrono::steady_clock::now() - switch_start_time_;
        RCLCPP_INFO(node_->get_logger(),
                    "Tile switch complete: %d (%.0f ms)", current_tile_,
                    std::chrono::duration<double, std::milli>(total_time)
                        .count());
        resetSwitch();
      }
      break;
    }

    case SwitchState::IDLE:
      break;
    }
  }

  void resetSwitch() {
    switch_state_ = SwitchState::IDLE;
    pending_tile_ = -1;
    last_switch_time_ = std::chrono::steady_clock::now();
  }

  // ========================================================================
  // Initialization
  // ========================================================================

  bool initialize() {
    // Use the shared bt_navigator node so service futures are processed by
    // its executor rather than a rogue node with no executor backing it.
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Load config
    std::string config_file;
    if (!getInput("config_file", config_file) || config_file.empty()) {
      try {
        std::string pkg_share =
            ament_index_cpp::get_package_share_directory("tile_manager");
        config_file = pkg_share + "/config/tiles_config.yaml";
      } catch (...) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot find tile_manager package");
        return false;
      }
    }

    if (!loadConfig(config_file)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load config: %s",
                   config_file.c_str());
      return false;
    }

    // Try to read persisted tile first, fall back to tile 1
    int persisted = readPersistedTile();
    if (persisted > 0) {
      current_tile_ = persisted;
      RCLCPP_INFO(node_->get_logger(),
                  "Restored tile from file: %d", current_tile_);
    } else {
      current_tile_ = 1;  // Default fallback
      RCLCPP_INFO(node_->get_logger(),
                  "No persisted tile found, defaulting to tile 1");
    }

    // Setup service clients
    map_loader_ =
        node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    costmap_clear_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(
        "/global_costmap/clear_entire_costmap");

    // Persist current tile (ensure file exists)
    persistTile(current_tile_);

    RCLCPP_INFO(
        node_->get_logger(),
        "TileCheckAction initialized: %zu tiles, %zu zones, current tile %d",
        tiles_.size(), trigger_zones_.size(), current_tile_);

    return true;
  }

  // ========================================================================
  // Config loading
  // ========================================================================

  bool loadConfig(const std::string &path) {
    try {
      YAML::Node config = YAML::LoadFile(path);

      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("tile_manager");
      std::string maps_dir = pkg_share + "/maps/";

      // ========================================
      // Parse tiles
      // ========================================
      if (config["tiles"]) {
        for (const auto &tile : config["tiles"]) {
          int id = tile.first.as<int>();
          std::string file = tile.second["file"].as<std::string>();
          tiles_[id] = maps_dir + file;
        }
      }

      // ========================================
      // Parse connections
      // ========================================
      if (config["connections"]) {
        for (const auto &conn : config["connections"]) {
          std::string key = conn.first.as<std::string>();
          auto data = conn.second;

          int tile_a, tile_b;
          if (!parseConnectionKey(key, tile_a, tile_b)) {
            RCLCPP_WARN(node_->get_logger(), "Invalid connection key: %s",
                        key.c_str());
            continue;
          }

          // Validate that both tiles exist
          if (tiles_.find(tile_a) == tiles_.end()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Connection '%s' references undefined tile %d",
                         key.c_str(), tile_a);
            continue;
          }
          if (tiles_.find(tile_b) == tiles_.end()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Connection '%s' references undefined tile %d",
                         key.c_str(), tile_b);
            continue;
          }

          // Parse overlap bounds
          auto overlap = data["overlap"];
          if (!overlap || overlap.size() != 4) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Connection '%s' has invalid overlap (need 4 values)",
                         key.c_str());
            continue;
          }

          double x_min = overlap[0].as<double>();
          double x_max = overlap[1].as<double>();
          double y_min = overlap[2].as<double>();
          double y_max = overlap[3].as<double>();

          std::string heading_axis = data["heading"].as<std::string>("x");

          // Validate heading axis
          if (heading_axis != "x" && heading_axis != "y") {
            RCLCPP_ERROR(node_->get_logger(),
                         "Connection '%s' has invalid heading '%s' (must be 'x' or 'y')",
                         key.c_str(), heading_axis.c_str());
            continue;
          }

          // Forward zone: tile_a -> tile_b
          TriggerZone tz_forward;
          tz_forward.x_min = x_min;
          tz_forward.x_max = x_max;
          tz_forward.y_min = y_min;
          tz_forward.y_max = y_max;
          tz_forward.from_tile = tile_a;
          tz_forward.to_tile = tile_b;
          tz_forward.heading = "+" + heading_axis;
          trigger_zones_.push_back(tz_forward);

          // Reverse zone: tile_b -> tile_a
          TriggerZone tz_reverse;
          tz_reverse.x_min = x_min;
          tz_reverse.x_max = x_max;
          tz_reverse.y_min = y_min;
          tz_reverse.y_max = y_max;
          tz_reverse.from_tile = tile_b;
          tz_reverse.to_tile = tile_a;
          tz_reverse.heading = "-" + heading_axis;
          trigger_zones_.push_back(tz_reverse);

          RCLCPP_DEBUG(node_->get_logger(),
                       "Connection %s: zone [%.2f,%.2f,%.2f,%.2f] axis=%s",
                       key.c_str(), x_min, x_max, y_min, y_max,
                       heading_axis.c_str());
        }
      }

      // ========================================
      // Parse settings (only cooldowns, no initial_tile)
      // ========================================
      if (config["settings"]) {
        switch_cooldown_ =
            config["settings"]["switch_cooldown"].as<double>(0.5);
        post_switch_delay_ =
            config["settings"]["post_switch_delay"].as<double>(0.3);
      }

      if (tiles_.empty()) {
        RCLCPP_WARN(node_->get_logger(),
                    "No tiles found in config '%s'. Tile switching is disabled.",
                    path.c_str());
      }

      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Config parse error: %s", e.what());
      return false;
    }
  }

  bool parseConnectionKey(const std::string &key, int &tile_a, int &tile_b) {
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

  // ========================================================================
  // Pose & heading
  // ========================================================================

  bool getRobotPose(double &x, double &y, double &yaw) {
    try {
      auto transform =
          tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

      x = transform.transform.translation.x;
      y = transform.transform.translation.y;

      auto &q = transform.transform.rotation;
      double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw = std::atan2(siny_cosp, cosy_cosp);

      return true;
    } catch (const tf2::TransformException &e) {
      return false;
    }
  }

  std::string yawToHeading(double yaw) {
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));

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

  // ========================================================================
  // Zone checking
  // ========================================================================

  int checkTriggerZone(double x, double y, const std::string &heading) {
    for (const auto &zone : trigger_zones_) {
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
};

}  // namespace tile_manager

#endif  // TILE_MANAGER__TILE_CHECK_ACTION_HPP_
