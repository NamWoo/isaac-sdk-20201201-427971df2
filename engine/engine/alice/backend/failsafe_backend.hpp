/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <atomic>
#include <map>
#include <string>

#include "engine/alice/backend/component_backend.hpp"
#include "engine/alice/components/Failsafe.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/scheduler/job_descriptor.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

class Failsafe;
class FailesafeHeartbeat;

// Provides a failsafe to all failsafe components
class FailsafeBackend : public ComponentBackend<Failsafe> {
 public:
  // Gives the current heartbeat status for a failsafe
  void notifyHeartbeatDeath(const std::string& failsafe_name, const std::string& heartbeat_name);
  // Reactivates a triggered failsafe
  void reactivate(const std::string& failsafe_name);

  void registerComponent(Failsafe* failsafe) override;
  void unregisterComponent(Failsafe* failsafe) override;
 private:
  std::map<std::string, Failsafe*> failsafes_;
};

// Provides a failsafe to all failsafe hearbeat components
class FailsafeHeartbeatBackend : public ComponentBackend<FailsafeHeartbeat> {
 public:
  FailsafeHeartbeatBackend(FailsafeBackend* failsafe_backend);

  void start() override;
  void stop() override;

  // Reanimates a triggered failsafe
  void reanimate(FailsafeHeartbeat* heartbeat);

  void registerComponent(FailsafeHeartbeat* heartbeat) override;
  void unregisterComponent(FailsafeHeartbeat* heartbeat) override;

 private:
  // Updates failsafe status for all hearbeats
  void update() const;

  FailsafeBackend* failsafe_backend_;

  std::map<Uuid, FailsafeHeartbeat*> heartbeats_;

  std::optional<scheduler::JobHandle> job_handle_;
};

}  // namespace alice
}  // namespace isaac
