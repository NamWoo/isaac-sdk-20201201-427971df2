/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "messages/geometry.hpp"
#include "messages/parking.capnp.h"

namespace isaac {
namespace message_generators {

// Publishes messages to simulate detected polyline.
// The provided prototype polyline is published on every tick().
class Polyline2Generator : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  // Output generated polyline
  ISAAC_PROTO_TX(Polyline2dProto, polyline);

  // The prototype of the generated polyline
  ISAAC_PARAM(std::vector<Vector2d>, prototype, {});
};

}  // namespace message_generators
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::message_generators::Polyline2Generator);
