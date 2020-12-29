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
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/alice/component.hpp"

namespace isaac {
namespace alice {

class FailsafeHeartbeatBackend;
class FailsafeBackend;

// A soft failsafe switch which can be used to check if a certain component of the system is
// still reactive. The failsafe is kept alive by a FailsafeHeartbeat component. Failsafe and
// FailsafeHeartbeat components can be in different nodes. They are identified via the `name`
// parameter.
class Failsafe : public Codelet {
 public:
  void initialize() override;
  void start() override;
  void stop() override;
  void deinitialize() override;

  // Checks if the failsafe is triggered
  bool isAlive() const {
    return is_alive_.load();
  }
  // Triggers the failsafe
  void stop_failsafe() {
    is_alive_ = false;
  }
  // Reactivates the failsafe
  void reactivate() {
    is_alive_ = true;
  }

  // the name of the failsafe
  ISAAC_PARAM(std::string, name);

 private:
  std::atomic<bool> is_alive_;
  FailsafeBackend* backend_;
};

// A soft heartbeat which can be used to keep a failsafe alive. If the heartbeat is not activated
// in time the corresponding failsafe will fail.
class FailsafeHeartbeat : public Component {
 public:
  void initialize() override;
  void deinitialize() override;

  // Checks if the heart is still beating
  bool isAlive() const;
  // Keeps the heart beating
  void beep();
  // Reanimates the failsafe to which this hearbeat is connected
  void reanimate();

  // The expected heart beat interval (in seconds). This is the time duration for which the
  // heartbeat will stay activated after a single activation. The heartbeat needs to be activated
  // again within this time interval, otherwise the corresponding Failsafe will fail.
  ISAAC_PARAM(double, interval)
  // The name of the failsafe to which this heartbeat is linked. This must be the same as the `name`
  // parameter in the corresponding Failsafe component.
  ISAAC_PARAM(std::string, failsafe_name);
  // The name of this heartbeat. This is purely for informative purposes.
  ISAAC_PARAM(std::string, heartbeat_name);

 private:
  FailsafeHeartbeatBackend* backend_;

  bool beeped_ = false;
  int64_t expected_beep_timestamp_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::Failsafe)
ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::FailsafeHeartbeat)
