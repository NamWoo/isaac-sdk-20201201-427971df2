/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/alice_codelet.hpp"
#include "messages/math.capnp.h"

namespace isaac {
namespace dummy {

class DummyPose2dProducer : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  ISAAC_PROTO_TX(Pose2dProto, pose)
};

class DummyPose2dConsumer : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  ISAAC_PROTO_RX(Pose2dProto, pose)

  ISAAC_PARAM(int, threshold, 5)
};

}  // namespace dummy
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::dummy::DummyPose2dProducer);
ISAAC_ALICE_REGISTER_CODELET(isaac::dummy::DummyPose2dConsumer);
