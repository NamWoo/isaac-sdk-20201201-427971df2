/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/gems/math/exponential_moving_average.hpp"

namespace isaac {
namespace alice {

class MessageLedger;

// Monitors messages flowing through a channel
class ChannelMonitor : public Codelet {
 public:
  void initialize() override;
  void start() override;
  void tick() override;

  // The name of the channel to be monitored
  ISAAC_PARAM(std::string, channel);

  // If enabled rates will be updated during tick. If the tick period is high compared to the
  // measured rate this will lead to jittering in the visualization.
  ISAAC_PARAM(bool, update_rate_on_tick, true);

 private:
  // An action executed when a message is received on the monitored channel.
  void onMessageRx(ConstMessageBasePtr message);
  // An action executed when a message is transmitted on the monitored channel.
  void onMessageTx(ConstMessageBasePtr message);

  MessageLedger* ledger_;
  math::ExponentialMovingAverageRate<double> rx_rate_;
  math::ExponentialMovingAverageRate<double> tx_rate_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::ChannelMonitor);
