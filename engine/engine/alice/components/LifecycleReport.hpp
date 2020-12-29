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

namespace isaac {
namespace alice {

// Periodically writes a report about node and component life cycle to the redis metadata server.
// FIXME
class LifecycleReport : public Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

 private:
  void sendStatusUpdates();

  int count_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::LifecycleReport)
