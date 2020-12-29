/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "redis_backend.hpp"

#include "engine/alice/application.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/logger.hpp"
#include "hiredis.h"

namespace isaac {
namespace alice {

RedisBackend::RedisBackend(Application* app) {
  app_uuid_str_ = app->uuid().str();

  struct timeval timeout = { 1, 500000 };  // 1.5 seconds
  context_ = redisConnectWithTimeout("localhost", 6379, timeout);
  if (context_ == nullptr || context_->err != REDIS_OK) {
    LOG_WARNING(R"(
**
**  Could not connect to redis server: %s.
**
**  Metadata will not be available for this run.
**
**  You can start a redis server by running: external\redis\redis-server
**)", context_ == nullptr ? "N/A" : context_->errstr);
    // Free the context and ignore commands
    if (context_ != nullptr) {
      redisFree(context_);
      context_ = nullptr;
    }
  } else {
    LOG_INFO("Successfully connected to Redis server.\n");
  }

  command("RPUSH isaac:runs %s", app_uuid_str_.c_str());
  command("HSET isaac:app:%s name %s", app_uuid_str_.c_str(), app->name().c_str());
}

RedisBackend::~RedisBackend() {
  if (context_ != nullptr) {
    redisFree(context_);
    context_ = nullptr;
  }
}

void RedisBackend::start() {
  command("HSET isaac:app:%s start_time %lld", app_uuid_str_.c_str(), NowCount());
}

void RedisBackend::stop() {
  command("HSET isaac:app:%s stop_time %lld", app_uuid_str_.c_str(), NowCount());
}

void RedisBackend::command(const char* command, ...) {
  if (context_ == nullptr) {
    return;
  }
  va_list args;
  va_start(args, command);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    redisReply* reply = static_cast<redisReply*>(redisvCommand(context_, command, args));
    if (reply == nullptr || reply->type == REDIS_REPLY_ERROR) {
      LOG_ERROR("Error sending command to redis server: %s", reply == nullptr ? "N/A" : reply->str);
    }
    if (reply != nullptr) {
      freeReplyObject(reply);
    }
  }
  va_end(args);
}

}  // namespace alice
}  // namespace isaac
