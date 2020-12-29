/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "NodeStatistics.hpp"

#include <string>
#include <utility>

#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

void NodeStatistics::start() {
  tickPeriodically();
}

void NodeStatistics::tick() {
  // Check if special statistics where requested
  bool send_full_graph = false;
  rx_request().processAllNewMessages([&](auto json, int64_t pubtime, int64_t acqtime) {
    const auto maybe_request = serialization::TryGetFromMap<std::string>(json, "request");
    if (!maybe_request) {
      LOG_ERROR("Invalid request: %s", json.dump(2).c_str());
      return;
    }
    send_full_graph = (*maybe_request == "graph");
  });

  // Send out statistics
  nlohmann::json msg;
  if (send_full_graph) {
    msg["graph"] = ApplicationJsonLoader::GetGraphJson(*node()->app());
  }
  msg["node_statistics"] = node()->app()->backend()->node_backend()->getNodeStatistics();
  msg["codelet_statistics"] = node()->app()->backend()->node_backend()->getCodeletStatistics();
  tx_statistics().publish(std::move(msg));
}

}  // namespace alice
}  // namespace isaac
