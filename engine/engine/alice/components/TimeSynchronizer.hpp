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
namespace alice {

// Helps time synchronization of messages.
// "app-clock" starts when the application starts. Therefore "app-clock"s of two different
// applications running at the same time will potentially vary substantially. To sync messages
// between two applications, we need to resort to a common clock. Currently, only time_since_epoch
// is supported as the common clock, which will be the same for applications running on same device.
// More "mode"s will be added later. Currently, only Tcp communication is synchronized. To sync
// messages that are sent to or received through network, simply add a TimeSynchronizer component to
// the node(s) with TcpPublishers and TcpSubscribers. Publisher will use syncToAppTime and
// Subscriber will use appToSyncTime. Messages within an app will be in "app-clock", but messages
// over network will be in "sync-clock".
class TimeSynchronizer : public Codelet {
 public:
  void start() override;

  // Given a time in app-clock, returns the corresponding time in sync-clock.
  int64_t appToSyncTime(int64_t app_time) const;
  // Given a time in sync-clock, returns the corresponding time in app-clock.
  int64_t syncToAppTime(int64_t sync_time) const;

 private:
  // How far our app is coming behind
  int64_t offset_time_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::TimeSynchronizer)
