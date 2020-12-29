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
#include "messages/led_strip.capnp.h"

namespace isaac {

// Simple codelet to cycle through red, green, and blue colors to test LED display
class SimpleLed : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // The outgoing LED strip message for the driver to display
  ISAAC_PROTO_TX(LedStripProto, led_strip);

 private:
  // Start on red and cycle through red - green - blue
  std::string fill_color_;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::SimpleLed);
