/*
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "MessageLedger.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <shared_mutex>  // NOLINT
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/message_ledger_backend.hpp"
#include "engine/alice/node.hpp"
#include "engine/gems/scheduler/scheduler.hpp"
#include "gxf/gems/staging_queue/staging_queue.hpp"

namespace isaac {
namespace alice {

namespace {
// Copies a text into a string and return the pointer of the next character to be written.
// If append_slash is true, it will add / at the end
char* AppendToString(const std::string& text, bool append_slash, char* ptr) {
  std::copy(text.begin(), text.end(), ptr);
  ptr += text.size();
  if (append_slash) (*ptr++) = '/';
  return ptr;
}
}  // namespace

void MessageLedger::initialize() {
  // Notify the scheduler about all received messages to trigger codelets
  addOnConnectAsRxCallback(
    [this](const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx) {
      this->addOnMessageCallback(rx, tx.component,
          [this, rx, tx](ConstMessageBasePtr message) {
            this->notifyScheduler(rx, message->pubtime);
          });
    });
}

void MessageLedger::deinitialize() {
  disconnect();
}

std::string MessageLedger::Endpoint::name() const {
  std::string str;
  const std::string& node_name = component->node()->name();
  const std::string& comp_name = component->name();
  // Preallocate the memory and perform the copy directly in place. (needs +2 for the /)
  str.resize(2 + tag.size() + comp_name.size() + node_name.size());
  char* ptr = AppendToString(node_name, true, &str[0]);
  ptr = AppendToString(comp_name, true, ptr);
  ptr = AppendToString(tag, false, ptr);
  return str;
}

std::string MessageLedger::Endpoint::nameWithApp() const {
  std::string str;
  const std::string& app_name = component->node()->app()->name();
  const std::string& node_name = component->node()->name();
  const std::string& comp_name = component->name();
  // Preallocate the memory and perform the copy directly in place. (needs +3 for the /)
  str.resize(3 + tag.size() + comp_name.size() + node_name.size() + app_name.size());
  char* ptr = AppendToString(app_name, true, &str[0]);
  ptr = AppendToString(node_name, true, ptr);
  ptr = AppendToString(comp_name, true, ptr);
  ptr = AppendToString(tag, false, ptr);
  return str;
}

void MessageLedger::addOnConnectAsTxCallback(OnConnectCallback callback) {
  on_connect_as_tx_callbacks_.emplace_back(std::move(callback));
}

void MessageLedger::addOnConnectAsRxCallback(OnConnectCallback callback) {
  on_connect_as_rx_callbacks_.emplace_back(std::move(callback));
}

void MessageLedger::provide(const Endpoint& endpoint, ConstMessageBasePtr message) {
  getOrCreateChannel(endpoint)->addMessage(message);
  // call general callbacks
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  for (auto& callback : callbacks_) {
    callback(endpoint, message);
  }
}

void MessageLedger::notifyScheduler(const Endpoint& endpoint, int64_t target_time) const {
  node()->app()->backend()->scheduler()->notify(endpoint.name(), target_time);
}

void MessageLedger::addOnMessageCallback(const Endpoint& endpoint, const Component* source,
                                         OnMessageCallback callback) {
  getOrCreateChannel(endpoint)->addCallback(source, std::move(callback));
}

void MessageLedger::addOnMessageCallback(OnChannelMessageCallback callback) {
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_.push_back(callback);
}

void MessageLedger::removeCustomer(const Component* source) {
  std::unique_lock<std::shared_timed_mutex> lock(channels_mutex_);
  for (auto& kvp : channels_) {
    kvp.second->removeSource(source);
  }
}

void MessageLedger::addSource(MessageLedger* source, const Component* customer) {
  std::unique_lock<std::mutex> lock(sources_mutex_);
  sources_.insert(std::make_pair(source, customer));
}

void MessageLedger::disconnect() {
  std::unique_lock<std::mutex> lock(sources_mutex_);
  for (auto& source : sources_) {
    source.first->removeCustomer(source.second);
  }
  sources_.clear();
}

void MessageLedger::iterateAllQueues(
    std::function<void(const Endpoint&, ConstMessageQueue&)> callback) const {
  std::shared_lock<std::shared_timed_mutex> lock(channels_mutex_);
  for (auto& kvp : channels_) {
    callback(kvp.first, kvp.second->messages());
  }
}

ConstMessageQueue& MessageLedger::getOrCreateQueue(const Endpoint& endpoint) {
  return getOrCreateChannel(endpoint)->messages();
}

ConstMessageQueue* MessageLedger::tryGetQueue(const Endpoint& endpoint) {
  if (const auto channel = findChannel(endpoint)) {
    return &(channel->messages());
  } else {
    return nullptr;
  }
}

size_t MessageLedger::numSourceChannels() const {
  return sources_.size();
}

std::map<std::string, MessageLedger::ChannelStatistics> MessageLedger::getStatistics() const {
  std::shared_lock<std::shared_timed_mutex> lock(channels_mutex_);
  std::map<std::string, ChannelStatistics> result;
  for (const auto& kvp : channels_) {
    result[full_name() + ":" + kvp.first.name()] = kvp.second->getStatistics();
  }
  return result;
}

MessageLedger::Channel::Channel(int capacity)
:  messages_(capacity, gxf::staging_queue::OverflowBehavior::kPop, ConstMessageBasePtr{}) {
}

void MessageLedger::Channel::addCallback(const Component* source, OnMessageCallback callback) {
  std::lock_guard<std::mutex> lock(channels_mutex_);
  callbacks_[source].emplace_back(std::move(callback));
}

void MessageLedger::Channel::removeSource(const Component* source) {
  std::lock_guard<std::mutex> lock(channels_mutex_);
  callbacks_.erase(source);
}

void MessageLedger::Channel::addMessage(ConstMessageBasePtr message) {
  // Append message
  {
    std::lock_guard<std::mutex> lock(messages_mutex_);
    statistics_.count++;
    statistics_.rate.add(1.0, ToSeconds(NowCount()));  // FIXME(dweikersdorf) Should use app clock
    messages_.push(message);
  }
  // Calls callbacks
  {
    std::lock_guard<std::mutex> lock(channels_mutex_);
    for (auto& kvp : callbacks_) {
      for (auto& callback : kvp.second) {
        callback(message);
      }
    }
  }
}

MessageLedger::ChannelStatistics MessageLedger::Channel::getStatistics() const {
  std::lock_guard<std::mutex> lock(messages_mutex_);
  return statistics_;
}

std::shared_ptr<MessageLedger::Channel> MessageLedger::findChannel(const Endpoint& endpoint) const {
  std::shared_lock<std::shared_timed_mutex> lock(channels_mutex_);
  auto it = channels_.find(endpoint);
  if (it == channels_.end()) {
    return nullptr;
  } else {
    return it->second;
  }
}

std::shared_ptr<MessageLedger::Channel> MessageLedger::getOrCreateChannel(
      const Endpoint& endpoint) {
  // Try to find the channel under a shared lock.
  channels_mutex_.lock_shared();
  auto it = channels_.find(endpoint);
  if (it != channels_.end()) {
    std::shared_ptr<Channel> sptr = it->second;
    channels_mutex_.unlock_shared();
    return sptr;
  } else {
    // We need to create the channel first. We have to release the shared lock and acquire a
    // unique lock.
    channels_mutex_.unlock_shared();
    channels_mutex_.lock();
    it = channels_.emplace(
        std::make_pair(endpoint, std::make_shared<Channel>(get_history()))).first;
    channels_mutex_.unlock();
    return it->second;
  }
}

void MessageLedger::connectAsTx(const Endpoint& tx, const Endpoint& rx) {
  for (auto& callback : on_connect_as_tx_callbacks_) {
    callback(tx, rx);
  }
}

void MessageLedger::connectAsRx(const Endpoint& tx, const Endpoint& rx) {
  for (auto& callback : on_connect_as_rx_callbacks_) {
    callback(tx, rx);
  }
}

}  // namespace alice
}  // namespace isaac
