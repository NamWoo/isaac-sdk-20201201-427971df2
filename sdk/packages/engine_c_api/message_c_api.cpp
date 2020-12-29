/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "message_c_api.hpp"

#include <string.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "capnp/compat/json.h"
#include "engine/alice/message.hpp"
#include "engine/core/buffers/algorithm.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "messages/proto_registry.hpp"

namespace isaac {
namespace alice {

MessageCApi::MessageCApi() {
  json_codec_ = std::make_unique<::capnp::JsonCodec>();
}

isaac_error_t MessageCApi::getJson(isaac_const_json_t* json) noexcept {
  if (json == nullptr) return isaac_error_invalid_parameter;

  // If we have a message pointer with a JSON payload; cache the payload on first call
  if (json_string_.empty() && message_ptr_) {
    auto* json_msg = dynamic_cast<const JsonMessage*>(message_ptr_.get());
    if (json_msg) {
      // Processes JsonMessage
      json_string_ = json_msg->data.dump();
    } else {
      // Processes ProtoMessage
      auto* proto_msg = dynamic_cast<const ProtoMessageBase*>(message_ptr_.get());
      if (!proto_msg) {
        return isaac_error_invalid_handle;
      }
      ::capnp::ReaderOptions options;
      options.traversalLimitInWords = kj::maxValue;
      ::capnp::SegmentArrayMessageReader reader(proto_msg->segments(), options);
      auto maybe_reader = GetRootReaderByTypeId(proto_msg->proto_id(), reader);
      if (!maybe_reader) {
        return isaac_error_invalid_message;
      }
      json_string_ = (::kj::StringPtr)json_codec_->encode(*maybe_reader);
    }
  }

  // Return pointer to the internal data
  // TODO These will be invalidated when writeJson is called.
  json->data = json_string_.data();
  json->size = json_string_.size() + 1;

  return isaac_error_success;
}

isaac_error_t MessageCApi::readJson(isaac_json_t* target_json) noexcept {
  if (target_json == nullptr) return isaac_error_invalid_parameter;

  // Get the JSON object
  isaac_const_json_t source_json;
  const isaac_error_t code = getJson(&source_json);
  if (code != isaac_error_success) return code;

  // Check if the buffer is valid to receive the JSON data
  const bool is_buffer_valid = target_json->size >= source_json.size;

  // Only copy data if buffer is valid
  if (is_buffer_valid) {
    std::copy(source_json.data, source_json.data + source_json.size, target_json->data);
    target_json->data[source_json.size - 1] = '\0';
  }

  // Always give size of JSON (either to confirm write, or to indicate number of required bytes)
  target_json->size = source_json.size;

  return is_buffer_valid ? isaac_error_success : isaac_error_data_not_read;
}

isaac_error_t MessageCApi::writeJson(const isaac_const_json_t* json) noexcept {
  if (message_ptr_) return isaac_error_cannot_modify_received_message;
  if (json == nullptr) return isaac_error_invalid_parameter;
  json_string_ = std::string(json->data);
  return isaac_error_success;
}

isaac_error_t MessageCApi::getBuffers(isaac_buffer_t* target_buffers, int64_t* target_buffer_count,
                                      isaac_memory_t preferred_storage) noexcept {
  if (target_buffer_count == nullptr) return isaac_error_invalid_parameter;

  const std::vector<SharedBuffer>& source_buffers = message_ptr_ ? message_ptr_->buffers : buffers_;
  const int64_t source_buffer_count = static_cast<int64_t>(source_buffers.size());

  // If target_buffers is null then we just want to get the number of buffer.
  if (target_buffers == nullptr) {
    *target_buffer_count = source_buffer_count;
    return isaac_error_success;
  }

  const bool is_buffer_too_small = *target_buffer_count < source_buffer_count;
  if (!is_buffer_too_small) {
    for (int64_t i = 0; i < source_buffer_count; i++) {
      const SharedBuffer& source = source_buffers[i];
      isaac_buffer_t& target = target_buffers[i];

      // Decide which in which storage mode the data will be presented
      const bool source_has_cuda = source.hasCudaStorage();
      const bool source_has_host = source.hasHostStorage();
      if (source_has_cuda && source_has_host) {
        target.storage = preferred_storage;
      } else if (source_has_host) {
        target.storage = isaac_memory_host;
      } else if (source_has_cuda) {
        target.storage = isaac_memory_cuda;
      } else {
        target.storage = isaac_memory_none;
      }

      // Get pointer and size for chosen storage order
      if (target.storage == isaac_memory_host) {
        target.pointer = source.host_buffer().begin();
        target.size = source.host_buffer().size();
      } else if (target.storage == isaac_memory_cuda) {
        target.pointer = source.cuda_buffer().begin();
        target.size = source.cuda_buffer().size();
      } else {
        target.pointer = nullptr;
        target.size = 0;
      }
    }
  }

  // Return the number of buffers
  *target_buffer_count = source_buffer_count;

  return is_buffer_too_small ? isaac_error_data_not_read : isaac_error_success;
}

isaac_error_t MessageCApi::appendBuffer(const isaac_buffer_t* buffer,
                                        int64_t* buffer_index) noexcept {
  if (message_ptr_) return isaac_error_cannot_modify_received_message;
  if (buffer == nullptr) return isaac_error_invalid_parameter;
  if (buffer->pointer == nullptr) return isaac_error_invalid_parameter;
  if (buffer->size < 0) return isaac_error_invalid_parameter;

  std::unique_ptr<SharedBuffer> message_buffer;

  if (buffer->storage == isaac_memory_host) {
    message_buffer = std::make_unique<SharedBuffer>(CpuBuffer(buffer->size));
    CopyArrayRaw(buffer->pointer, isaac::BufferStorageMode::Host,
                 message_buffer->host_buffer().begin(), isaac::BufferStorageMode::Host,
                 buffer->size);
  } else if (buffer->storage == isaac_memory_cuda) {
    message_buffer = std::make_unique<SharedBuffer>(CudaBuffer(buffer->size));
    CopyArrayRaw(buffer->pointer, isaac::BufferStorageMode::Cuda,
                 message_buffer->cuda_buffer().begin(), isaac::BufferStorageMode::Cuda,
                 buffer->size);
  } else {
    return isaac_error_unknown_memory_type;
  }

  if (buffer_index) {
    // index will be the size before we add the new buffer
    *buffer_index = buffers_.size();
  }

  buffers_.emplace_back(std::move(*message_buffer));
  return isaac_error_success;
}

isaac_error_t MessageCApi::getUuid(uint64_t* upper, uint64_t* lower) noexcept {
  if (!upper || !lower) {
    return isaac_error_invalid_parameter;
  }

  *upper = uuid_.upper();
  *lower = uuid_.lower();
  return isaac_error_success;
}

isaac_error_t MessageCApi::setUuid(uint64_t upper, uint64_t lower) noexcept {
  uuid_ = Uuid::FromUInt64(lower, upper);
  return isaac_error_success;
}

isaac_error_t MessageCApi::getAcqtime(int64_t* time) noexcept {
  if (!time) {
    return isaac_error_invalid_parameter;
  }

  *time = acqtime_;
  return isaac_error_success;
}
isaac_error_t MessageCApi::setAcqtime(int64_t time) noexcept {
  if (message_ptr_) {
    return isaac_error_cannot_modify_received_message;
  }

  acqtime_ = time;
  return isaac_error_success;
}

isaac_error_t MessageCApi::getPubtime(int64_t* time) noexcept {
  if (!time) {
    return isaac_error_invalid_parameter;
  }

  *time = pubtime_;
  return isaac_error_success;
}

isaac_error_t MessageCApi::setPubtime(int64_t time) noexcept {
  if (message_ptr_) {
    return isaac_error_cannot_modify_received_message;
  }

  pubtime_ = time;
  return isaac_error_success;
}

isaac_error_t MessageCApi::getProtoId(int64_t* proto_id) noexcept {
  if (!proto_id) {
    return isaac_error_invalid_parameter;
  }

  *proto_id = proto_id_;
  return isaac_error_success;
}

isaac_error_t MessageCApi::setProtoId(int64_t proto_id) noexcept {
  if (message_ptr_) {
    return isaac_error_cannot_modify_received_message;
  }

  proto_id_ = proto_id;
  return isaac_error_success;
}

isaac_error_t MessageCApi::readProtoBytes(const void** segments_data, uint64_t* sizes,
                                          uint64_t* num_segments) noexcept {
  // Checks if a message is received
  if (!message_ptr_) {
    return isaac_error_message_not_found;
  }
  if (!num_segments) {
    return isaac_error_invalid_parameter;
  }
  // Checks if the message is ProtoMessage
  auto* proto_msg = dynamic_cast<const ProtoMessageBase*>(message_ptr_.get());
  if (!proto_msg) {
    return isaac_error_invalid_handle;
  }

  constexpr uint16_t word_length = sizeof(capnp::word);
  const auto segments = proto_msg->segments();
  const uint64_t recv_num_segments = segments.size();
  if (segments_data != nullptr && sizes != nullptr && *num_segments >= recv_num_segments) {
    for (uint64_t i = 0; i < recv_num_segments; ++i) {
      segments_data[i] = reinterpret_cast<const void*>(segments[i].begin());
      sizes[i] = segments[i].size() * word_length;
    }
  }
  *num_segments = segments.size();

  return isaac_error_success;
}

isaac_error_t MessageCApi::setProtoBytes(const void** segments_data, const int64_t* sizes,
                                         const int64_t num_segments) noexcept {
  if (message_ptr_) {
    return isaac_error_cannot_modify_received_message;
  }
  if (num_segments < 0) {
    return isaac_error_invalid_parameter;
  }
  constexpr uint16_t word_length = sizeof(capnp::word);
  for (int64_t i = 0; i < num_segments; ++i) {
    if (sizes[i] % word_length != 0) {
      return isaac_error_invalid_parameter;
    }
  }
  proto_segments_.resize(num_segments);

  for (int64_t i = 0; i < num_segments; ++i) {
    const int64_t segment_size = sizes[i];
    const void* segment_ptr = segments_data[i];
    if (segment_ptr == nullptr || segment_size < 0) {
      proto_segments_.clear();
      return isaac_error_invalid_parameter;
    }
    proto_segments_[i].resize(segment_size);
    std::memcpy(proto_segments_[i].pointer().get(), segment_ptr, segment_size);
  }
  return isaac_error_success;
}

isaac_error_t MessageCApi::setConvertFlag(isaac_message_convert_t convert_flag) noexcept {
  if (message_ptr_) {
    return isaac_error_cannot_modify_received_message;
  }
  convert_flag_ = convert_flag;
  return isaac_error_success;
}

}  // namespace alice
}  // namespace isaac
