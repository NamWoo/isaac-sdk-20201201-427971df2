/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <mutex>
#include <set>
#include <shared_mutex>  // NOLINT
#include <string>
#include <unordered_map>

#include "engine/alice/component.hpp"
#include "engine/alice/components/MessageLedger.hpp"

namespace isaac {
namespace alice {

// @internal
// This component is keeping track which sight channels are currently active.
class SightChannelStatus : public Component {
 public:
  // Used by the sight backend to know whether or not some channels are listening to.
  bool isChannelActive(const MessageLedger::Endpoint& endpoint);

  // Used by a Sight visualization server to notify that someone is listening on a channel
  void addChannelListener(const std::string& name);

  // Used by a Sight visualization server to notify that someone has stopped listening to a  channel
  void removeChannelListener(const std::string& name);

  // Returns the list of currently existing channels.
  std::set<std::string> getListChannels();

 private:
  // Helper function to update a channel. It increments the counter by inc.
  void updateChannelHelper(const std::string& name, int inc);

  std::shared_timed_mutex mutex_;
  std::unordered_map<const Component*, std::map<std::string, int>> channel_listeners_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::SightChannelStatus)
