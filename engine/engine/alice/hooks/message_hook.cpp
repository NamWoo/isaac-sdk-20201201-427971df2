/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "message_hook.hpp"

#include <locale>
#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/message_ledger_backend.hpp"
#include "engine/alice/components/Codelet.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

MessageHook::MessageHook(Component* component, const std::string& tag)
    : Hook(component), tag_(tag) {}

void MessageHook::connect() {
  ledger_ = component()->node()->getComponent<MessageLedger>();
  RxMessageHook* rx = dynamic_cast<RxMessageHook*>(this);
  if (rx) {
    Codelet* codelet = dynamic_cast<Codelet*>(rx->component());
    ASSERT(codelet, "Currently only codelets support RX message hooks");
    codelet->addRx(rx);

    rx->queue_ = &(ledger_->getOrCreateQueue({component(), tag()}));
  }
}

std::string MessageHook::channel_id() const {
  return MessageLedger::Endpoint{component(), tag()}.name();
}

void RxMessageHook::processAllNewMessagesImpl(
    std::function<void(const ConstMessageBasePtr&)> callback) const {
  for (const auto& message : queue()) {
    callback(message);
  }
  queue().popAll();
}

void TxMessageHook::publishImpl(MessageBasePtr message, std::optional<int64_t> acqtime) {
  const int64_t now = component()->node()->clock()->timestamp();
  message->uuid = Uuid::Generate();
  message->acqtime = acqtime ? *acqtime : now;
  message->pubtime = now;
  message->buffers = std::move(buffers_);
  buffers_.clear();
  ledger()->provide({component(), tag()}, message);
}

void Connect(Component* tx, const std::string& tx_tag, Component* rx, const std::string& rx_tag) {
  ASSERT(tx->node()->app()->uuid() == rx->node()->app()->uuid(),
         "Components to be connected must be in the same application");
  tx->node()->app()->backend()->message_ledger_backend()->connect({tx, tx_tag}, {rx, rx_tag});
}

void Connect(Component* tx, const std::string& tx_tag, const std::string& rx_channel) {
  // Split rx_channel in the form "node/component/tag" into ("node/component", "tag")
  const auto split = tx->node()->app()->getComponentAndTag(rx_channel);
  tx->node()->app()->backend()->message_ledger_backend()->connect(
      {tx, tx_tag}, {std::get<0>(split), std::get<1>(split)});
}

bool IsValidChannelName(const std::string& str) {
  const std::locale locale("C");
  for (const char c : str) {
    if (!std::isalnum(c, locale) && c != '_') return false;
  }
  return true;
}

}  // namespace alice
}  // namespace isaac
