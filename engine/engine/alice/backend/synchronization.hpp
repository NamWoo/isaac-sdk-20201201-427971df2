/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "engine/alice/message.hpp"
#include "engine/gems/algorithm/timeseries.hpp"

namespace isaac {
namespace alice {

// Keeps track of messages per channel and finds synchronized pairs
// Currently a semi-efficient algorithm is used and only exact matching by acqtime is supported.
// Timeseries are also limited to at most 20 messages which means that if channels are too far
// out of sync they will not synchronize.
// TODO Implement inexact matching and make the max length configurable.
class ChannelSynchronizer {
 public:
  // merges a synchronizer into this one. This will also empty synchronizer
  void merge(ChannelSynchronizer&& synchronizer);
  // marks a channels as required for the matching
  void mark(const std::string& channel);
  // adds a message for a channel
  void push(const std::string& channel, alice::ConstMessageBasePtr message);
  // pops the next synchronized tuple and stores it in `sync`; returns true if one was available
  bool sync_pop(std::map<std::string, alice::ConstMessageBasePtr>& sync);
  // returns true if the given channel is part of this synchronizer
  bool contains(const std::string& channel) const;

 private:
  std::map<std::string, Timeseries<alice::ConstMessageBasePtr, int64_t>> channels_;
};

}  // namespace alice
}  // namespace isaac
