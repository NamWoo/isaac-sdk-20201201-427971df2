/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "messages/utils/utils.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/gems/serialization/capnp.hpp"

namespace isaac {
namespace utils {
alice::MessageBasePtr CreateProtoMessage(const Uuid uuid, const uint64_t type_id,
                                         CpuBuffer&& proto_bytes,
                                         std::vector<CpuBuffer>&& buffers) {
  if (proto_bytes.size() == 0 || type_id == 0) {
    // Fails on invalid proto data
    return nullptr;
  }

  // Creates and populates proto message payload
  std::vector<uint8_t> proto_buffer;
  std::vector<size_t> segment_lengths;
  isaac::serialization::FlatArrayToCapnpBuffer(std::move(proto_bytes), segment_lengths,
                                               proto_buffer);
  auto message_base =
      std::make_shared<alice::BufferedProtoMessage>(proto_buffer, 0, segment_lengths);

  // Populates proto id
  message_base->type = type_id;
  // Generates UUID
  message_base->uuid = uuid;

  // Populates buffers
  message_base->buffers.reserve(buffers.size());
  for (auto& buffer : buffers) {
    message_base->buffers.emplace_back(std::move(buffer));
  }

  return message_base;
}

alice::MessageBasePtr CreateProtoMessage(const Uuid uuid, const uint64_t type_id,
                                         std::vector<CpuBuffer>&& segments,
                                         std::vector<CpuBuffer>&& buffers) {
  if (segments.size() == 0 || type_id == 0) {
    // Fails on invalid proto data
    return nullptr;
  }
  auto message_base = std::make_shared<alice::BufferedProtoMessage>(std::move(segments));

  // Populates proto id
  message_base->type = type_id;
  // Generates UUID
  message_base->uuid = uuid;

  // Populates buffers
  message_base->buffers.reserve(buffers.size());
  for (auto& buffer : buffers) {
    message_base->buffers.emplace_back(std::move(buffer));
  }

  return message_base;
}

}  // namespace utils
}  // namespace isaac
