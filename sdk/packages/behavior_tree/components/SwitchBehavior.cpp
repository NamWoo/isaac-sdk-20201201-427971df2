/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SwitchBehavior.hpp"

#include <string>

namespace isaac {
namespace behavior_tree {

void SwitchBehavior::start() {
  // Copy the alias/node names association map into the cache variable.
  node_alias_map_ = get_node_alias_map();

  // Try to start the desired behavior
  current_behavior_ = try_get_desired_behavior();
  if (!current_behavior_) {  // no behavior is selected, then we already succeed
    reportSuccess();
    return;
  }

  if (current_behavior_->empty()) {
    reportFailure("Empty string for desired behavior");
    return;
  }

  const std::string& resolved_node = getAlias(*current_behavior_);
  if (resolved_node.empty()) {
    reportFailure("Empty string for aliased node");
    return;
  }

  alice::Node* child = findChildByName(resolved_node);
  if (!child) {
    reportFailure("No child or alias with name '%s'", current_behavior_->c_str());
    return;
  }

  // Got a valid child node, set actual current behavior and start it.
  current_behavior_ = resolved_node;
  startChild(*child);

  // FIXME: Ideally we would tick also if the configuration changes so we can switch behavior
  //  while running.
  tickOnChildStatus();
}

void SwitchBehavior::tick() {
  const auto status = getChildStatus(*current_behavior_);
  if (status != alice::Status::RUNNING) {
    updateStatus(status, "child status changed");
  }
}

const std::string& SwitchBehavior::getAlias(const std::string& choice) {
  const auto it = node_alias_map_.find(choice);
  return (it != node_alias_map_.end()) ? it->second : choice;
}

}  // namespace behavior_tree
}  // namespace isaac
