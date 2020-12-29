/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <string>

#include "engine/core/optional.hpp"
#include "packages/behavior_tree/components/Behavior.hpp"

namespace isaac {
namespace behavior_tree {

// @experimental
// A behavior which executes exactly one of its children, selected by name.
class SwitchBehavior : public Behavior {
 public:
  // Used to map aliases to actual node names.
  using NodeAliasMap = std::map<std::string, std::string>;

  void start() override;
  void tick() override;

  // The name of the child node with the desired behavior. Only the initial value is used, i.e.,
  // switching while this behavior is already running is not supported.
  ISAAC_PARAM(std::string, desired_behavior);
  // Map from alias to node name for conveniently abstracting away from the actual node name. This
  // variable is read and cached internally during `start()`. Aliases can be configured this way:
  // "node_alias_map": {
  //   "alias1": "nodenameA",
  //   "alias2": "nodenameB",
  //   "alias3": "nodenameC"
  // }
  ISAAC_PARAM(NodeAliasMap, node_alias_map, {});

 private:
  // Tries to resolve the given `choice` to an alias name specified in the `node_alias_map_` cache
  // of the `node_alias_map` parameter. Returns the entry of that alias if it is present, and
  // `choice` otherwise.
  const std::string& getAlias(const std::string& choice);

  // The behavior which is currently running
  std::optional<std::string> current_behavior_;
  // Cache variable holding the alias/node names association valid during start.
  NodeAliasMap node_alias_map_;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::SwitchBehavior);
