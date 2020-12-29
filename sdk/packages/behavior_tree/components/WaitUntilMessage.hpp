/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace behavior_tree {

// Reports success upon receiving a message at the parameterized channel name. This behavior codelet
// can be used to create more sophisticated behavior trees such as:
// 1. Do not move the car until camera images are received,
// 2. Slow down the car upon detecting a parking spot.
class WaitUntilMessage : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Name of input channel
  ISAAC_PARAM(std::string, channel_name, "message");

 private:
  // Message hook for receiving message
  std::unique_ptr<alice::RxMessageHook> hook_;
};

}  // namespace behavior_tree
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::behavior_tree::WaitUntilMessage)
