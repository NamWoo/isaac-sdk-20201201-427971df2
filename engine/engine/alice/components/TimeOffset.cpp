/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "TimeOffset.hpp"

#include <string>
#include <vector>

#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

void TimeOffset::initialize() {
  ledger_ = node()->getComponent<MessageLedger>();
  ledger_->addOnConnectAsRxCallback(
      [=](const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx) {
        if (rx.tag == get_input_channel()) {
          ledger_->addOnMessageCallback(rx, tx.component,
              [=](ConstMessageBasePtr message) {
                auto clone = Clone(message);
                if (!clone) {
                  LOG_ERROR("Could not clone message");
                  return;
                }
                clone->acqtime += get_acqtime_offset();
                ledger_->provide({this, get_output_channel()}, clone);
              });
          return;
        }
      });
}

}  // namespace alice
}  // namespace isaac
