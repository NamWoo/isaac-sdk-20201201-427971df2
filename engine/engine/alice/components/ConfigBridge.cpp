/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ConfigBridge.hpp"

#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

void ConfigBridge::start() {
  tickOnMessage(rx_request());
}

void ConfigBridge::tick() {
  // Parse incoming message
  const auto& json = rx_request().get();
  const auto maybe_request = serialization::TryGetFromMap<std::string>(json, "request");
  if (!maybe_request) {
    LOG_ERROR("Request does not contain 'request': %s", json.dump(2).c_str());
    return;
  }
  if (*maybe_request == "get") {
    Json reply;
    reply["config"] = node()->app()->backend()->config_backend()->root();
    tx_reply().publish(std::move(reply));
  } else if (*maybe_request == "set") {
    const auto maybe_data = serialization::TryGetFromMap<Json>(json, "config");
    if (!maybe_data) {
      LOG_ERROR("Request 'set' does not contain 'data': %s", json.dump(2).c_str());
      return;
    }
    node()->app()->backend()->config_backend()->set(*maybe_data);
  } else {
    LOG_ERROR("Unknown request: %s", maybe_request->c_str());
  }
}

}  // namespace alice
}  // namespace isaac
