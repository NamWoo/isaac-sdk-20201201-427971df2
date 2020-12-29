/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "blob.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "engine/core/assert.hpp"

namespace isaac {
namespace serialization {

size_t AccumulateLength(const std::vector<ByteArrayConstView>& blobs) {
  return AccumulateLength(blobs.data(), blobs.size());
}

size_t AccumulateLength(const ByteArrayConstView* pointer, size_t size) {
  size_t length = 0;
  for (size_t i = 0; i < size; i++) {
    length += pointer[i].size();
  }
  return length;
}

byte* CopyAll(const std::vector<ByteArrayConstView>& blobs, byte* dst, byte* dst_end) {
  return CopyAll(blobs.data(), blobs.size(), dst, dst_end);
}

byte* CopyAll(const ByteArrayConstView* pointer, size_t size, byte* dst, byte* dst_end) {
  for (size_t i = 0; i < size; i++) {
    const ByteArrayConstView* buffer = pointer + i;
    ASSERT(buffer != nullptr, "Buffer must not be null");
    const size_t length = buffer->size();
    ASSERT(dst + length <= dst_end, "Out of bounds");
    const size_t actual_source_length = buffer->end() - buffer->begin();
    ASSERT(actual_source_length == length, "Invalid buffer");
    std::copy(buffer->begin(), buffer->end(), dst);
    dst += length;
  }
  return dst;
}

void BlobsToLengths32u(const std::vector<ByteArrayConstView>& blobs,
                       std::vector<uint32_t>& lengths) {
  lengths.resize(blobs.size());
  for (size_t i = 0; i < blobs.size(); i++) {
    const size_t length = blobs[i].size();
    ASSERT(length <= std::numeric_limits<uint32_t>::max(),
           "Blob to big to store its length as a 32-bit unsigned (length: %zu)", length);
    lengths[i] = static_cast<uint32_t>(length);
  }
}

void CapnpArraysToBlobs(const kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments,
                        std::vector<ByteArrayConstView>& blobs) {
  blobs.resize(segments.size());
  for (size_t i = 0; i < blobs.size(); i++) {
    auto segment = segments[i].asBytes();
    blobs[i] = ByteArrayConstView{segment.begin(), segment.size()};
  }
}

}  // namespace serialization
}  // namespace isaac
