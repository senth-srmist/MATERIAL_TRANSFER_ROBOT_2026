/**
 * @file tile_check_plugin.cpp
 * @brief Register TileCheckAction as a Nav2 BT plugin
 * 
 * Note: We explicitly define BT_RegisterNodesFromPlugin with proper
 * visibility attributes instead of using BT_REGISTER_NODES macro,
 * which may define it as static in some configurations.
 */

#include "tile_manager/tile_check_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

// Explicitly export the registration function with C linkage and default visibility
extern "C" {
  void __attribute__((visibility("default"))) 
  BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
  {
    factory.registerNodeType<tile_manager::TileCheckAction>("TileCheckAction");
  }
}
