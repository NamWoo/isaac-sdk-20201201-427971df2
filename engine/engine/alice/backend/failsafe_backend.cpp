/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "failsafe_backend.hpp"

#include <string>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/components/Failsafe.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/scheduler/scheduler.hpp"

namespace isaac {
namespace alice {

void FailsafeBackend::notifyHeartbeatDeath(const std::string& failsafe_name,
                                           const std::string& heartbeat_name) {
  const auto it = failsafes_.find(failsafe_name);
  if (it == failsafes_.end()) {
    return;  // FIXME the thread starts before onStart is called and the failssafe is registered
  }
  ASSERT(it != failsafes_.end(), "No failsafe with name '%s'", failsafe_name.c_str());
  Failsafe* failsafe = it->second;
  if (failsafe->isAlive()) {
    failsafe->stop_failsafe();
    LOG_WARNING("Failsafe triggered for '%s' because heartbeat '%s' stopped beating",
                failsafe_name.c_str(), heartbeat_name.c_str());
  }
}

void FailsafeBackend::reactivate(const std::string& failsafe_name) {
  const auto it = failsafes_.find(failsafe_name);
  if (it == failsafes_.end()) {
    // Node with the failsafe may not have started yet.
    // Do not print to keep the console clean.
    return;
  }
  if (!it->second->isAlive()) {
    LOG_INFO("Reanimating failsafe '%s'", failsafe_name.c_str());
    it->second->reactivate();
  }
}

void FailsafeBackend::registerComponent(Failsafe* failsafe) {
  // This needs to be in the start otherwise get_name is not available
  ASSERT(failsafe, "argument null");
  const std::string failsafe_name = failsafe->get_name();
  ASSERT(failsafes_.find(failsafe_name) == failsafes_.end(),
         "A failsafe with name %s was already created", failsafe_name.c_str());
  failsafes_[failsafe_name] = failsafe;
}

void FailsafeBackend::unregisterComponent(Failsafe* failsafe) {
  failsafes_.erase(failsafe->get_name());
}

FailsafeHeartbeatBackend::FailsafeHeartbeatBackend(FailsafeBackend* failsafe_backend)
    : failsafe_backend_(failsafe_backend) {}

void FailsafeHeartbeatBackend::start() {
  scheduler::JobDescriptor job_descriptor;
  job_descriptor.priority = 0;
  job_descriptor.period = SecondsToNano(0.01);
  job_descriptor.execution_mode = scheduler::ExecutionMode::kPeriodicTask;
  job_descriptor.name = "FailsafeHeartBeat";
  job_descriptor.action = [this] {
    update();
  };
  job_handle_ = app()->backend()->scheduler()->createJobAndStart(job_descriptor);
  ASSERT(job_handle_, "Unable to create Failsafe Heartbeat");
}

void FailsafeHeartbeatBackend::stop() {
  app()->backend()->scheduler()->destroyJobAndWait(*job_handle_);
}

void FailsafeHeartbeatBackend::reanimate(FailsafeHeartbeat* heartbeat) {
  failsafe_backend_->reactivate(heartbeat->get_failsafe_name());
}

void FailsafeHeartbeatBackend::registerComponent(FailsafeHeartbeat* heartbeat) {
  ASSERT(heartbeat, "argument null");
  heartbeats_[heartbeat->uuid()] = heartbeat;
}

void FailsafeHeartbeatBackend::unregisterComponent(FailsafeHeartbeat* heartbeat) {
  heartbeats_.erase(heartbeat->uuid());
}

void FailsafeHeartbeatBackend::update() const {
  for (const auto& kvp : heartbeats_) {
    const FailsafeHeartbeat* heartbeat = kvp.second;
    if (!heartbeat->isAlive()) {
      failsafe_backend_->notifyHeartbeatDeath(heartbeat->get_failsafe_name(),
                                              heartbeat->get_heartbeat_name());
    }
  }
}

}  // namespace alice
}  // namespace isaac
