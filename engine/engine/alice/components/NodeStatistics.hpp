/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

// @internal
// Collects statistics about nodes into a JSON object and publishes them at a certain frequency
class NodeStatistics : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Input channel for requests to send certain detail statistics
  ISAAC_RAW_RX(nlohmann::json, request);
  // Output channel for statistics based on JSON
  ISAAC_RAW_TX(nlohmann::json, statistics);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::NodeStatistics);
