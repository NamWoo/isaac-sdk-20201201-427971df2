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
#include <string>
#include <utility>
#include <vector>

#include "capnp/compat/json.h"

#include "engine/core/buffers/buffer.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "isaac_c_api.h"

namespace isaac {
class SharedBuffer;

namespace alice {

class BufferCApi;
class MessageBase;

// Implementation of the Isaac engine Message C-API. Please refer to isaac_c_api.h for details.
//
// This class is only intended for use with the ApplicationCApi class and
// its constructors reflect this fact. There should be no attempt to use
// this class outside of the c-api interface.
class MessageCApi {
 public:
  MessageCApi();
  ~MessageCApi() = default;

  MessageCApi(const MessageCApi&) = delete;
  MessageCApi(MessageCApi&&) = delete;
  MessageCApi& operator=(const MessageCApi&) = delete;
  MessageCApi& operator=(MessageCApi&&) = delete;

  isaac_error_t getJson(isaac_const_json_type* json) noexcept;
  isaac_error_t readJson(isaac_json_type* json) noexcept;
  isaac_error_t writeJson(const isaac_const_json_type* json) noexcept;

  isaac_error_t getBuffers(isaac_buffer_t* buffers, int64_t* buffer_count,
                           isaac_memory_t preferred_storage) noexcept;

  isaac_error_t appendBuffer(const isaac_buffer_t* buffer, int64_t* buffer_index) noexcept;

  isaac_error_t getUuid(uint64_t* upper, uint64_t* lower) noexcept;
  isaac_error_t setUuid(uint64_t upper, uint64_t lower) noexcept;

  isaac_error_t getAcqtime(int64_t* time) noexcept;
  isaac_error_t setAcqtime(int64_t time) noexcept;

  isaac_error_t getPubtime(int64_t* time) noexcept;
  isaac_error_t setPubtime(int64_t time) noexcept;

  isaac_error_t getProtoId(int64_t* proto_id) noexcept;
  isaac_error_t setProtoId(int64_t proto_id) noexcept;

  isaac_error_t readProtoBytes(const void** segments_data, uint64_t* sizes,
                               uint64_t* num_segments) noexcept;
  isaac_error_t setProtoBytes(const void** segments_data, const int64_t* sizes,
                              const int64_t num_segments) noexcept;

  isaac_error_t setConvertFlag(isaac_message_convert_t proto_flag) noexcept;

 private:
  // This is part of the ApplicationCApi in spirit. It is broken out for practical
  // utilization however.
  friend class ApplicationCApi;

  // pointer to a received message if one exists.
  std::shared_ptr<const MessageBase> message_ptr_;

  // Array of buffers associated with the message.
  std::vector<SharedBuffer> buffers_;

  // Storage for the message as a json body
  std::string json_string_;
  // Storage for the Proto bytes
  std::vector<CpuBuffer> proto_segments_;

  // Uuid identifies the message
  Uuid uuid_;
  // Acquisition time of the message in nanoseconds
  int64_t acqtime_{0};
  // Publication time of the message in nanoseconds
  int64_t pubtime_{0};
  // The proto id of the message
  int64_t proto_id_{0};
  // Codec for Proto<->Json conversion
  std::unique_ptr<::capnp::JsonCodec> json_codec_;
  // Flag for publishing as JsonMessage or ProtoMessage
  isaac_message_convert_t convert_flag_{isaac_message_type_json};
};

}  // namespace alice
}  // namespace isaac
