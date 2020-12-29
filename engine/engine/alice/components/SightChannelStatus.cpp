/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SightChannelStatus.hpp"

#include <set>
#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/node.hpp"
#include "engine/gems/algorithm/string_utils.hpp"

namespace isaac {
namespace alice {

bool SightChannelStatus::isChannelActive(const alice::MessageLedger::Endpoint& endpoint) {
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    const auto& comp_it = channel_listeners_.find(endpoint.component);
    if (comp_it != channel_listeners_.end()) {
      const auto& component_map = comp_it->second;
      const auto tag_it = component_map.find(endpoint.tag);
      if (tag_it != component_map.end()) {
        return tag_it->second > 0;
      }
    }
  }
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  // Create the object if it does not exist, but do not change the value if it does.
  channel_listeners_[endpoint.component][endpoint.tag];
  // For the first message we let it go through to notify the channel exist
  // TODO implement a better way to notify of a new channel
  return true;
}

void SightChannelStatus::updateChannelHelper(const std::string& name, int inc) {
  const auto splits = SplitString(name, '/');
  // Check the channel belong to this app, if not we can skip, otherwise we update it.
  if (splits.size() != 4 || splits[0] != node()->app()->name()) return;
  const auto* node_ptr = node()->app()->findNodeByName(splits[1]);
  if (node_ptr == nullptr) return;
  const auto* comp_ptr = node_ptr->findComponentByName(splits[2]);
  if (comp_ptr == nullptr) return;
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  channel_listeners_[comp_ptr][splits[3]] += inc;
}

void SightChannelStatus::addChannelListener(const std::string& name) {
  updateChannelHelper(name, 1);
}

void SightChannelStatus::removeChannelListener(const std::string& name) {
  updateChannelHelper(name, -1);
}

std::set<std::string> SightChannelStatus::getListChannels() {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  std::set<std::string> tags;
  for (const auto& comp_it : channel_listeners_) {
    for (const auto& tag_it : comp_it.second) {
      // Append the name of the app
      tags.insert(std::move((MessageLedger::Endpoint{comp_it.first, tag_it.first}).nameWithApp()));
    }
  }
  return tags;
}

}  // namespace alice
}  // namespace isaac
