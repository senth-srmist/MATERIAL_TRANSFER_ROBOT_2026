// Copyright (c) 2026 Tejas
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_depth_obstacle_layer/bt_nodes/human_blocking_path.hpp"
#include "nav2_depth_obstacle_layer/bt_nodes/wait_until_human_clears.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

// BehaviorTree.CPP v3 plugin registration
// Using extern "C" for proper symbol export

extern "C" {

void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<nav2_depth_obstacle_layer::HumanBlockingPath>(
    "HumanBlockingPath");
  factory.registerNodeType<nav2_depth_obstacle_layer::WaitUntilHumanClears>(
    "WaitUntilHumanClears");
}

}  // extern "C"
