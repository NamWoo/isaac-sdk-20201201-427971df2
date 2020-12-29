/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "message_ledger_backend.hpp"

#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

void MessageLedgerBackend::connect(const MessageLedger::Endpoint& tx,
                                   const MessageLedger::Endpoint& rx) {
  ASSERT(tx.component != nullptr, "argument null");
  auto* tx_ledger = tx.component->node()->getComponentOrNull<MessageLedger>();
  ASSERT(tx_ledger != nullptr, "Publisher node does not have a MessageLedger component");
  ASSERT(rx.component != nullptr, "argument null");
  auto* rx_ledger = rx.component->node()->getComponentOrNull<MessageLedger>();
  ASSERT(rx_ledger != nullptr, "Publisher node does not have a MessageLedger component");
  // Each message arriving in the TX are provided to the RX
  tx_ledger->addOnMessageCallback(tx, rx.component,
      [=](ConstMessageBasePtr message) {
        rx_ledger->provide(rx, message);
      });
  // Call the on-connection callbacks to setup further on-message calls
  tx_ledger->connectAsTx(tx, rx);
  rx_ledger->connectAsRx(tx, rx);
  // Remember the TX as a source so that the connection can be removed later
  rx_ledger->addSource(tx_ledger, rx.component);
  // store connection
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_.push_back({tx.name(), rx.name()});
  }
}

}  // namespace alice
}  // namespace isaac
