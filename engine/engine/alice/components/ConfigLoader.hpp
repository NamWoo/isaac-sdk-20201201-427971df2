/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/components/Codelet.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

// At start, sets the config given as a parameter and reports success.
// TODO: Support $(fullname <>) syntax to apply prefix
class ConfigLoader : public Codelet {
 public:
  void start() override;

  // The config blob to be written in format like
  // {"node_foo":{"component_bar":{"key_0":val_0, ... } } }
  ISAAC_PARAM(nlohmann::json, config, {});
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::ConfigLoader)
