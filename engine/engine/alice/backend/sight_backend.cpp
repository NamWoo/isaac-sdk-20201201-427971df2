/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "sight_backend.hpp"

#include <memory>
#include <string>
#include <utility>

#include "engine/alice/component.hpp"
#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/components/SightChannelStatus.hpp"

namespace isaac {
namespace alice {

void SightBackend::show(const Component* component, const std::string& name, double time,
                        std::function<void(sight::Sop&)> callback) {
  if (!isActive(component, name)) {
    return;
  }
  const int64_t start_timestamp = NowCount();
  sight::Sop sop;
  callback(sop);
  const int64_t stop_timestamp = NowCount();
  publishSightMessageImpl(component, name, time, std::move(sop));
  collectStatistics(component->full_name(), name, stop_timestamp - start_timestamp);
}

void SightBackend::show(const Component* component, const std::string& name, double time,
                        sight::Sop sop) {
  if (!isActive(component, name)) {
    return;
  }
  publishSightMessageImpl(component, name, time, std::move(sop));
}

void SightBackend::show(Component* component, Json json) {
  if (json.find("name") == json.end()) {
    LOG_ERROR("Invalid Sight Json: Name field missing");
    return;
  }
  const std::string name = json["name"];
  if (!isActive(component, name)) {
    return;
  }
  publishSightMessageImpl(component, name, std::move(json));
}

bool SightBackend::isActive(const Component* component, const std::string& name) {
  for (auto channel_status : channel_status_) {
    if (channel_status->isChannelActive({component, name})) {
      return true;
    }
  }
  return false;
}

void SightBackend::registerCustomer(MessageLedger* message_ledger) {
  ASSERT(message_ledger != nullptr, "Message ledger is null");
  message_ledgers_.emplace_back(message_ledger);
}

void SightBackend::registerChannelStatus(SightChannelStatus* component) {
  ASSERT(component != nullptr, "Channel status is null");
  channel_status_.emplace_back(component);
}

void SightBackend::publishSightMessageImpl(const Component* component, const std::string& name,
                                           Json json) {
  auto message = std::make_shared<SightMessage>();
  message->json = std::move(json);
  // Distribute message to listeners
  for (auto ledger : message_ledgers_) {
    ledger->provide({component, name}, message);
  }
}

void SightBackend::publishSightMessageImpl(const Component* component, const std::string& name,
                                           double time, sight::Sop sop) {
  Json json;
  json["type"] = "sop";
  json["v"] = sop.moveJson();
  json["t"] = time;
  publishSightMessageImpl(component, name, std::move(json));
  collectStatistics(component->full_name(), name);
}

void SightBackend::collectStatistics(const std::string& component, const std::string& tag,
                                     int64_t duration) {
  // TODO implement
}

void SightBackend::collectStatistics(const std::string& component, const std::string& tag) {
  // TODO implement
}

}  // namespace alice
}  // namespace isaac
