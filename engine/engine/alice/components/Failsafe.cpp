/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Failsafe.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/failsafe_backend.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/time.hpp"

namespace isaac {
namespace alice {

void Failsafe::initialize() {
  backend_ = node()->app()->backend()->getBackend<FailsafeBackend>();
}

void Failsafe::start() {
  // This needs to be in the start otherwise get_name is not available
  backend_->registerComponent(this);
}

void Failsafe::stop() {
  // This needs to be in the stop otherwise get_name is not available
  backend_->unregisterComponent(this);
}

void Failsafe::deinitialize() {
  backend_ = nullptr;
}

void FailsafeHeartbeat::initialize() {
  backend_ = node()->app()->backend()->getBackend<FailsafeHeartbeatBackend>();
  backend_->registerComponent(this);
}

void FailsafeHeartbeat::deinitialize() {
  backend_->unregisterComponent(this);
  backend_ = nullptr;
}

bool FailsafeHeartbeat::isAlive() const {
  if (!beeped_) {
    return false;
  }
  const int64_t now = node()->clock()->timestamp();
  return now <= expected_beep_timestamp_;
}

void FailsafeHeartbeat::beep() {
  const int64_t now = node()->clock()->timestamp();
  expected_beep_timestamp_ = now + SecondsToNano(get_interval());
  beeped_ = true;
}

void FailsafeHeartbeat::reanimate() {
  beep();
  backend_->reanimate(this);
}

}  // namespace alice
}  // namespace isaac
