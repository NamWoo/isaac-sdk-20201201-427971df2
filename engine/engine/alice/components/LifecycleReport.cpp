/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "LifecycleReport.hpp"

#include <string>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/node_backend.hpp"
#include "engine/alice/backend/redis_backend.hpp"

namespace isaac {
namespace alice {

void LifecycleReport::start() {
  count_ = 0;
  tickPeriodically(0.25);
}

void LifecycleReport::tick() {
  sendStatusUpdates();
}

void LifecycleReport::stop() {
  sendStatusUpdates();
}

void LifecycleReport::sendStatusUpdates() {
  const std::string app_str = node()->app()->uuid().str();

  RedisBackend& redis = *node()->app()->backend()->redis_backend();

  const int old_count = count_;
  for (const auto& update : node()->app()->backend()->node_backend()->flushStatusUpdates()) {
    redis.command("HMSET isaac:%s:lifecycle:%05d ts %lld node %s old %d new %d",
                  app_str.c_str(), count_, update.timestamp, update.node_name.c_str(),
                  static_cast<int>(update.old_stage), static_cast<int>(update.new_stage));
    if (!update.component_name.empty()) {
      redis.command("HSET isaac:%s:lifecycle:%05d component %s", app_str.c_str(), count_,
                    update.component_name.c_str());
    }
    count_++;
  }
  if (count_ != old_count) {
    redis.command("SET isaac:%s:lifecycle:count %d", app_str.c_str(), count_);
  }
}

}  // namespace alice
}  // namespace isaac
