/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <list>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "engine/core/logger.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/math/exponential_moving_average.hpp"

namespace isaac {

// Helper scheduler:
// Given a target bandwidth and a flow of messages on different channel, this class helps to decide
// whether or not we should drop messages such as we stay within 5% of the target bandwidth, and
// such as each channel get throttled at the same frequency (and not the same bandwidth).
// For example if the bandwidth is limited at 1Mb/s and we have two channels one outputing 10kb
// messages at 100Hz and the other 100kb messages at 10Hz. Both channel will be throttled at around
// 9.1Hz (9.1 * (100k + 10k) ~ 1M).
// WARNING: This class is not thread safe, each message needs to be processed sequentially.
template <class Key>
class FlowControl {
 public:
  FlowControl(double target_bandwidth = 0.0) {
    target_bandwidth_ = target_bandwidth;
    target_frequency_ = -1.0;
  }

  // Resets the target bandwidth.
  void resetTargetBandwith(double target_bandwidth) {
    target_bandwidth_ = target_bandwidth;
    updateTargetFrequency();
  }

  // Returns whether or not we should process the message.
  bool keepMessage(const Key& channel_name, int64_t timestamp, int64_t size) {
    return keepMessage(channel_name, ToSeconds(timestamp), size);
  }

  // Returns whether or not we should process the message.
  bool keepMessage(const Key& channel_name, double time, int64_t size) {
    Channel& channel = getChannel(channel_name);
    channel.add(time, size);
    bool kept;
    all_channels_.add(time, size);
    if (target_frequency_ < 0.0 ||
        channel.frequency(time) - channel.skippedFrequency(time) < target_frequency_) {
      kept = true;
      all_channels_sent_.add(time, size);
    } else {
      kept = false;
      channel.addSkipped(time);
    }
    const double bandwidth = all_channels_sent_.bandwidth(time);
    if (bandwidth < kMinThreshold * target_bandwidth_ ||
        bandwidth > target_bandwidth_ * kMaxThreshold) {
      updateTargetFrequency();
    }
    return kept;
  }

  // Returns the current bandwidth of all the channels.
  double bandwidth(double time) {
    return all_channels_sent_.bandwidth(time);
  }

 private:
  static constexpr double kTimeWindows = 2.5;
  // We will adjust the max frequency when current bandwidth is greater than kMaxThreshold * target
  static constexpr double kMaxThreshold = 1.05;
  // We will adjust the max frequency when current bandwidth is less than kMinThreshold * target
  static constexpr double kMinThreshold = 0.95;

  // Helper to keep track of different frequency and bandwidth of each channel.
  class Channel {
   public:
    Channel() : frequency_(kTimeWindows), skipped_(kTimeWindows), bandwidth_(kTimeWindows) {}

    // Add a new message to the history
    void add(double time, int64_t size) {
      frequency_.add(1.0, time);
      bandwidth_.add(static_cast<double>(size), time);
      skipped_.updateTime(time);
    }

    // Records a new message that has been skipped
    void addSkipped(double time) {
      skipped_.add(1.0, time);
    }

    // Returns the frequency of messages on this channel
    double frequency(double time) {
      frequency_.updateTime(time);
      return frequency_.rate();
    }

    // Returns the frequency of messages skipped
    double skippedFrequency(double time) {
      skipped_.updateTime(time);
      return skipped_.rate();
    }

    // Returns the current bandwidth this channel require (if not throttled)
    double bandwidth(double time) {
      bandwidth_.updateTime(time);
      return bandwidth_.rate();
    }

    // Returns the time of the last update made to this channel.
    double getLastUpdate() const {
      return frequency_.time();
    }

   protected:
    math::ExponentialMovingAverageRate<double> frequency_;
    math::ExponentialMovingAverageRate<double> skipped_;
    math::ExponentialMovingAverageRate<double> bandwidth_;
  };

  // Returns a reference to the channel (creates a new channel if it did not exist).
  Channel& getChannel(const Key& channel) {
    if (channels_.find(channel) == channels_.end()) {
      channels_.insert({channel, Channel()});
    }
    return channels_[channel];
  }

  // Updates the target frequency (fastest rate at wich each channel can be processed without
  // exceeding the target bandwidth)
  void updateTargetFrequency() {
    const double time = all_channels_.getLastUpdate();
    // How much bandwidth we need to save.
    double target_save = all_channels_.bandwidth(time) - target_bandwidth_;
    // We have enough bandwidth for every messages
    if (target_save < 0.0) {
      target_frequency_ = -1.0;
      return;
    }
    // <frequency, bandwidth>
    std::vector<std::pair<double, double>> channels;
    for (auto& channel : channels_) {
      channels.push_back({channel.second.frequency(time), channel.second.bandwidth(time)});
    }
    // We can sort by decreasing frequency and process the channels in order until we have decreased
    // the target of enough channel to reach the desired bandwidth.
    std::sort(channels.rbegin(), channels.rend());
    double sum_message_size = channels[0].second / channels[0].first;
    size_t index = 1;
    target_frequency_ = channels[0].first;
    while (true) {
      // If we do not have channel, we can easily compute the optimal frequency.
      if (index == channels.size()) {
        target_frequency_ -= target_save / sum_message_size;
        break;
      }
      if (target_save / sum_message_size <= target_frequency_ - channels[index].first) {
        target_frequency_ -= target_save / sum_message_size;
        break;
      }
      // Update how much we need to save once we adjust the move the target frequency to the
      // frequency of the currently selected channel.
      target_save -= (target_frequency_ - channels[index].first) * sum_message_size;
      target_frequency_ = channels[index].first;
      // We need to adjust the sum of message size by adding the average message side of the current
      // channel.
      sum_message_size += channels[index].second / channels[index].first;
      index++;
    }
  }

  // The target frequency the channel can tick at.
  double target_frequency_;
  // The bandwidth we are trying to maitain within 5%.
  double target_bandwidth_;
  // Helper to get the current bandwidth if all messages were processed
  Channel all_channels_;
  // Helper to get the current bandwidth of processed messages.
  Channel all_channels_sent_;
  // Information relative to each channel needed in order to compute the target frequency.
  std::unordered_map<Key, Channel> channels_;
};

}  // namespace isaac
