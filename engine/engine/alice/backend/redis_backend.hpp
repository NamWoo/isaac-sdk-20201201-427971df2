/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "hiredis.h"

namespace isaac {
namespace alice {

class Application;

// This backend manages the connection to the redis server and provides a thread-safe function to
// write a command to the server. In case a connection to the server can not be established a
// warning is printed once and commands are ignored.
class RedisBackend  {
 public:
  RedisBackend(Application* app);
  ~RedisBackend();

  void start();
  void stop();

  // Sends a command to the redis server
  void command(const char* command, ...);

 private:
  // The UUID of the app as a string
  std::string app_uuid_str_;

  // Protects the redis context against concurrent access
  std::mutex mutex_;

  // The redis context
  redisContext* context_;
};

}  // namespace alice
}  // namespace isaac
