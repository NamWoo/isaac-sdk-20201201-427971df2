/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "capnp.hpp"

#include <cstring>
#include <string>
#include <vector>

#include "engine/core/assert.hpp"

namespace isaac {
namespace serialization {

namespace {
constexpr uint16_t word_length = sizeof(capnp::word);
}

void FlatArrayToCapnpBuffer(CpuBuffer&& bytes, std::vector<size_t>& segment_lengths,
                            std::vector<uint8_t>& buffer) {
  const uint32_t n_segments = *reinterpret_cast<const uint32_t*>(bytes.begin()) + 1;
  segment_lengths.resize(n_segments);
  uint64_t offset = 4;
  for (uint32_t i = 0; i < n_segments; i++) {
    segment_lengths[i] = *reinterpret_cast<const uint32_t*>(bytes.begin() + offset) *
                         static_cast<uint32_t>(word_length);
    offset += static_cast<uint64_t>(4);
  }
  // compute the offset. It can be either 4 or 0
  offset = (offset - static_cast<uint64_t>(1)) / static_cast<uint64_t>(word_length) *
               static_cast<uint64_t>(word_length) +
           static_cast<uint64_t>(word_length);
  buffer.assign(bytes.begin() + offset, bytes.end());
}

void CapnpSegmentsToFlatArray(const kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments,
                              CpuBuffer& bytes) {
  auto kj_array = ::capnp::messageToFlatArray(segments);
  auto byte_array = kj_array.asBytes();
  bytes.resize(byte_array.size());
  std::memcpy(const_cast<unsigned char*>(bytes.begin()), byte_array.begin(), byte_array.size());
}

}  // namespace serialization
}  // namespace isaac
