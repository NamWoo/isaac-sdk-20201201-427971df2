/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace alice {

// Runs specific command via std::system on start and stop.
class Subprocess : public alice::Codelet {
 public:
  void start() override;
  void stop() override;

  // The command to run on start
  ISAAC_PARAM(std::string, start_command);

  // The command to run on stop
  ISAAC_PARAM(std::string, stop_command);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::Subprocess);
