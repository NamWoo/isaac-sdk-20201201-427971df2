/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "synchronization.hpp"

#include <algorithm>
#include <map>
#include <string>
#include <utility>

namespace isaac {
namespace alice {

namespace {
constexpr int kMaxTimeseriesLength = 20;
}  // namespace

void ChannelSynchronizer::merge(ChannelSynchronizer&& synchronizer) {
  for (auto& it : synchronizer.channels_) {
    ASSERT(channels_.find(it.first) == channels_.end(),
           "Merging ChannelSynchronizer is only supported if they have an empty intersection. %s");
    channels_[it.first] = std::move(it.second);
  }
}

void ChannelSynchronizer::mark(const std::string& channel) {
  channels_[channel];
}

void ChannelSynchronizer::push(const std::string& channel, alice::ConstMessageBasePtr message) {
  auto it = channels_.find(channel);
  ASSERT(it != channels_.end(), "Can not push a message to this synchronizer. Use mark first.");
  auto& series = it->second;
  if (!series.empty() && message->acqtime <= series.youngest().stamp) {
    series.insert(message->acqtime, message);
  } else {
    series.push(message->acqtime, message);
  }
  series.forgetBySize(kMaxTimeseriesLength);
}

bool ChannelSynchronizer::sync_pop(std::map<std::string, alice::ConstMessageBasePtr>& sync) {
  sync.clear();
  if (channels_.empty()) {
    return false;
  }
  // compute maximum of oldest timestamp
  int64_t lower = 0;
  for (const auto& kvp : channels_) {
    if (kvp.second.empty()) return false;
    lower = std::max(lower, kvp.second.oldest().stamp);
  }
  // discard everything older than that timestamp
  for (auto& kvp : channels_) {
    kvp.second.rejuvenate(lower);
  }
  // Iterate over all samples in the first channel and check if we find a match in the other
  // channels. If we found a match erase it from all channels.
  auto channels_begin = channels_.begin();
  auto& hist0 = channels_begin->second;
  channels_begin++;
  for (size_t i = 0; i < hist0.size(); i++) {
    const int64_t timestamp = hist0.at(i).stamp;
    // check if it also exists in the other timestamps
    const bool is_a_match = std::all_of(channels_begin, channels_.end(),
        [timestamp](const auto kvp) {
          return kvp.second.find(timestamp) != -1;
        });
    // if we found a match remove it and return it
    if (is_a_match) {
      for (auto& kvp : channels_) {
        sync[kvp.first] = kvp.second.at(kvp.second.find(timestamp)).state;
        kvp.second.erase(timestamp);
      }
      return true;
    }
  }
  return false;
}

bool ChannelSynchronizer::contains(const std::string& channel) const {
  return channels_.find(channel) != channels_.end();
}

}  // namespace alice
}  // namespace isaac
