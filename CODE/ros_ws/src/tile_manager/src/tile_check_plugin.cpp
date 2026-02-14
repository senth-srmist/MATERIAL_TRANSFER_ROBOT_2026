/**
 * @file tile_check_plugin.cpp
 * @brief Register TileCheckAction as a BT plugin
 */

#include "tile_manager/tile_check_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

// Register the node with BehaviorTree.CPP
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tile_manager::TileCheckAction>("TileCheckAction");
}
