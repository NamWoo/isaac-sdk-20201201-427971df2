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

namespace isaac {
namespace dummy {

class DummyStatus : public alice::Codelet {
 public:
  void start() override;
  void tick() override;

  ISAAC_PARAM(int, status, 0);
};

}  // namespace dummy
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::dummy::DummyStatus);
