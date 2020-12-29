/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/components/Codelet.hpp"

namespace isaac {
namespace alice {

// @internal
// Exposes the configuration to a JSON-based interface for example for websight
class ConfigBridge : public Codelet {
 public:
  void start() override;
  void tick() override;

  // Request to the config system
  ISAAC_RAW_RX(nlohmann::json, request);
  // Reply from the config system
  ISAAC_RAW_TX(nlohmann::json, reply);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::ConfigBridge);
