/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <set>
#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/components/Codelet.hpp"
#include "engine/gems/pose_tree/pose_tree.hpp"

namespace isaac {
namespace alice {

// @internal
// Communication Bridge between the PoseTree and InteractiveMarkersManager
class InteractiveMarkersBridge : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Request to sight
  ISAAC_RAW_RX(nlohmann::json, request);
  // Reply to sight
  ISAAC_RAW_TX(nlohmann::json, reply);

 private:
  // Validate an incoming message from the Front end
  bool isMessageValid(const Json& json, const std::string& expected_cmd,
      const std::string& expected_param);
  // Load and validate the config file
  bool loadConfigFile();
  // Fill the list of the editable edges from config file
  void registerEdges();
  // Send the list of editable edges to Front end
  void sendEditableList();
  // Check if this edge is in our list of editable edges
  bool isEdgeEditable(const std::string& lhs, const std::string& rhs) const;

  // list of editable edges.
  std::set<std::pair<std::string, std::string>> edges_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::InteractiveMarkersBridge);
