// Copyright 2026 Farmience
// SPDX-License-Identifier: Apache-2.0

#include "behaviortree_cpp_v3/bt_factory.h"

#include "human_detection/human_blocking_path.hpp"
#include "human_detection/raise_human_alarm.hpp"
#include "human_detection/wait_human_clears.hpp"

extern "C" {

void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<human_detection::HumanBlockingPath>(
      "HumanBlockingPath");
  factory.registerNodeType<human_detection::RaiseHumanAlarm>("RaiseHumanAlarm");
  factory.registerNodeType<human_detection::WaitUntilHumanClears>(
      "WaitUntilHumanClears");
}

} // extern "C"
