/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ChannelMonitor.hpp"

#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

void ChannelMonitor::initialize() {
  ledger_ = node()->getComponent<MessageLedger>();
  ledger_->addOnConnectAsRxCallback(
      [=](const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx) {
        if (rx.tag == get_channel()) {
          ledger_->addOnMessageCallback(rx, tx.component,
              [=](ConstMessageBasePtr message) { onMessageRx(message); });
        }
      });
  ledger_->addOnConnectAsTxCallback(
      [=](const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx) {
        if (tx.tag == get_channel()) {
          ledger_->addOnMessageCallback(tx, rx.component,
              [=](ConstMessageBasePtr message) { onMessageTx(message); });
        }
      });
}

void ChannelMonitor::start() {
  tickPeriodically();
  rx_rate_ = math::ExponentialMovingAverageRate<double>(1.0);
  tx_rate_ = math::ExponentialMovingAverageRate<double>(1.0);
}

void ChannelMonitor::tick() {
  if (get_update_rate_on_tick()) {
    const double time = stopwatch().read();
    rx_rate_.updateTime(time);
    tx_rate_.updateTime(time);
  }
  show("rx_rate", rx_rate_.rate());
  show("tx_rate", tx_rate_.rate());
}

void ChannelMonitor::onMessageRx(ConstMessageBasePtr signal_message) {
  rx_rate_.add(1.0f, stopwatch().read());
}

void ChannelMonitor::onMessageTx(ConstMessageBasePtr signal_message) {
  tx_rate_.add(1.0f, stopwatch().read());
}

}  // namespace alice
}  // namespace isaac
