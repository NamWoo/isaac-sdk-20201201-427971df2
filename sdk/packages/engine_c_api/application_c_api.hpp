/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>

#include "isaac_c_api.h"

#include "capnp/compat/json.h"

#include "engine/alice/application.hpp"
#include "engine/alice/message.hpp"

namespace isaac {
namespace alice {

class MessageCApi;
class BufferCApi;

// Implementation of the Isaac engine Application C-API. Please refer to isaac_c_api.h for details.
//
// This class acts as a factory for self contained application handles

class ApplicationCApi {
 public:
  static ApplicationCApi* CreateApplicationCApi(const char* asset_path, const char* app_filename,
                                                const char** modules_path, int num_module_paths,
                                                const char** json_files, int num_json_file);

  ~ApplicationCApi() = default;

  ApplicationCApi(const ApplicationCApi&) = delete;
  ApplicationCApi(ApplicationCApi&&) = delete;
  ApplicationCApi& operator=(const ApplicationCApi&) = delete;
  ApplicationCApi& operator=(ApplicationCApi&&) = delete;

  isaac_error_t startApplication() noexcept;
  isaac_error_t stopApplication() noexcept;

  isaac_error_t createMessage(isaac_uuid_t* uuid) noexcept;
  isaac_error_t destroyMessage(const isaac_uuid_t* uuid) noexcept;
  isaac_error_t getMessage(const isaac_uuid_t* uuid, MessageCApi** message) noexcept;

  isaac_error_t publishMessage(const char* node_name, const char* component_name,
                               const char* channel_name, const isaac_uuid_t* uuid) noexcept;
  isaac_error_t receiveNewMessage(const char* node_name, const char* component_name,
                                  const char* channel_name, isaac_uuid_t* uuid) noexcept;
  isaac_error_t releaseMessage(isaac_uuid_t* uuid) noexcept;

  isaac_error_t getTime(int64_t* time) noexcept;

  isaac_error_t setPose(const char* lhs, const char* rhs, isaac_pose_t pose, int64_t time) noexcept;
  isaac_error_t getPose(const char* lhs, const char* rhs, int64_t time,
                        isaac_pose_t* pose) noexcept;

  isaac_error_t getParameter(const char* node, const char* component, const char* key,
                             isaac_json_t* json) noexcept;
  isaac_error_t setParameter(const char* node, const char* component, const char* key,
                             const isaac_const_json_t* json) noexcept;

  isaac_error_t getTimeDifference(double external_time, int64_t* difference);

 private:
  ApplicationCApi() = default;

  // Helper function for constructing messages with the isaac message callback readers
  isaac_error_t generateMessageHandle(ConstMessageBasePtr new_message, ApplicationCApi* app,
                                      isaac_uuid_t* uuid) noexcept;

  // Store the application
  std::unique_ptr<Application> app_;

  // Store the messages, by UUID
  std::unordered_map<Uuid, std::unique_ptr<MessageCApi>> uuid_to_message_map_;
  std::mutex uuid_to_message_map_mutex_;

  // Codec for Proto<->Json conversion
  std::unique_ptr<::capnp::JsonCodec> json_codec_;
};

}  // namespace alice
}  // namespace isaac
