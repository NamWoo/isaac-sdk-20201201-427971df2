/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "MessagePassingReport.hpp"

#include <string>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/backend/redis_backend.hpp"

namespace isaac {
namespace alice {

void MessagePassingReport::start() {
  tickPeriodically(3.0);
}

void MessagePassingReport::tick() {
  const std::string app_str = node()->app()->uuid().str();
  RedisBackend& redis = *node()->app()->backend()->redis_backend();
  for (Node* node : node()->app()->backend()->node_backend()->nodes()) {
    const MessageLedger* ledger = node->getComponentOrNull<MessageLedger>();
    if (ledger == nullptr) {
      continue;
    }
    for (const auto& kvp : ledger->getStatistics()) {
      redis.command("HSET isaac:%s:msgs:%s count %lld", app_str.c_str(), kvp.first.c_str(),
                    kvp.second.count);
      redis.command("HSET isaac:%s:msgs:%s rate %f", app_str.c_str(), kvp.first.c_str(),
                    kvp.second.rate.rate());
    }
  }
}

}  // namespace alice
}  // namespace isaac
