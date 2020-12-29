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
#include <utility>
#include <vector>

#include "engine/alice/message.hpp"
#include "engine/core/time.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/sight/sop.hpp"

namespace isaac {
namespace alice {

class Component;
class MessageLedger;
class SightChannelStatus;

// Manages sight visualization for alice. All sight API calls are stored in message ledgers
// provided by listeners.
class SightBackend {
 public:
  // Show operation for variables
  template <typename T, std::enable_if_t<!sight::is_sop_callback_v<T>, int> = 0>
  void show(const Component* component, const std::string& name, double time,
            const T& value) {
    if (!isActive(component, name)) {
      return;
    }
    Json json;
    json["type"] = "plot";
    json["t"] = time;
    json["v"] = value;
    publishSightMessageImpl(component, name, std::move(json));
  }
  // Show operation based on a callback
  void show(const Component* component, const std::string& name, double time,
            std::function<void(sight::Sop&)> callback);
  // Show operation based on an existing SOP
  void show(const Component* component, const std::string& name, double time, sight::Sop sop);
  // Show operation based on a JSON object
  void show(Component* component, Json json);

  // Returns true if data from this source would actually be displayed by sight
  bool isActive(const Component* component, const std::string& name);

  // Registers a message ledger customer
  void registerCustomer(MessageLedger* message_ledger);
  // Registers a message channel status component
  void registerChannelStatus(SightChannelStatus* component);

 private:
  // Creates a sight message and publishes it to all listeners
  void publishSightMessageImpl(const Component* component, const std::string& name, Json json);
  // Creates a sight message and publishes it to all listeners
  void publishSightMessageImpl(const Component* component, const std::string& name, double time,
                               sight::Sop sop);

  // Collects statistics on callback performance
  void collectStatistics(const std::string& component, const std::string& tag, int64_t duration);
  // Collects statistics
  void collectStatistics(const std::string& component, const std::string& tag);

  // List of all message ledgers which want to receive sight messages
  std::vector<MessageLedger*> message_ledgers_;
  // List of components which are queried to determine if data on a sight channel should be shown.
  std::vector<SightChannelStatus*> channel_status_;
};

}  // namespace alice
}  // namespace isaac
