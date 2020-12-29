/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/backend/stopwatch.hpp"
#include "engine/alice/component.hpp"

namespace isaac {
namespace alice {

class MessageLedger;

// Throttles messages on a data channel.
// If `use_signal_channel` is enabled a signal channel is used as a heartbeat. Messages on the data
// channel will only be published whenever a message on the signal channel was received.
// In any case `minimum_interval` is used to additionally throttle the output.
class Throttle : public Component {
 public:
  void initialize() override;

  // The name of the data channel to be throttled
  ISAAC_PARAM(std::string, data_channel);
  // The name of the output data channel with throttled data
  ISAAC_PARAM(std::string, output_channel);
  // The minimal time period after which a message can be published again on the data channel.
  ISAAC_PARAM(double, minimum_interval, 0.0);
  // If enabled the signal channel will define which incoming messages are passed on. This enables
  // the parameters `signal_channel` and `acqtime_tolerance`.
  ISAAC_PARAM(bool, use_signal_channel, true);
  // The name of the signal channel used for throttling
  ISAAC_PARAM(std::string, signal_channel);
  // The tolerance on the acqtime to match data and signal channels. If this parameter is not
  // specified the latest available message on the data channel will be taken.
  ISAAC_PARAM(int, acqtime_tolerance);

 private:
  // Called whenever a message on the data channel is received
  void onDataMessage(ConstMessageBasePtr message);
  // Called whenever a message on the signal channel is received
  void onSignalMessage(ConstMessageBasePtr message);
  // Called to publish a message on the output channel
  void publish(ConstMessageBasePtr message);

  MessageLedger* ledger_;
  Stopwatch stopwatch_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Throttle)
