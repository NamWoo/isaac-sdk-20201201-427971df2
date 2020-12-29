/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "header.hpp"

#include <algorithm>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace serialization {

namespace {

using CountT = uint8_t;

template <typename I>
uint8_t* WriteToBuffer(uint8_t* buffer, uint8_t* end, I value) {
  uint8_t* buffer_next = buffer + sizeof(I);
  if (end != nullptr) {
    if (buffer_next > end) return nullptr;
    // TODO endianess?
    *reinterpret_cast<I*>(buffer) = value;
  }
  return buffer_next;
}

template <typename I>
const uint8_t* ReadFromBuffer(const uint8_t* buffer, const uint8_t* end, I& value) {
  const uint8_t* buffer_next = buffer + sizeof(I);
  if (buffer_next > end) return nullptr;
  // TODO endianess?
  value = *reinterpret_cast<const I*>(buffer);
  return buffer_next;
}

uint8_t* WriteToBuffer(uint8_t* buffer, uint8_t* end, const Uuid& value) {
  buffer = WriteToBuffer(buffer, end, value.lower());
  return WriteToBuffer(buffer, end, value.upper());
}

const uint8_t* ReadFromBuffer(const uint8_t* buffer, const uint8_t* end, Uuid& value) {
  uint64_t lower, upper;
  buffer = ReadFromBuffer(buffer, end, lower);
  if (buffer == nullptr) return nullptr;
  buffer = ReadFromBuffer(buffer, end, upper);
  if (buffer == nullptr) return nullptr;
  value = Uuid::FromUInt64(lower, upper);
  return buffer;
}

template <typename I, typename C>
uint8_t* WriteContainerToBuffer(uint8_t* buffer, uint8_t* end, const C& source) {
  static_assert(std::is_same<I, typename C::value_type>::value, "value types mismatch");
  const size_t src_count = source.size();
  if (src_count > kHeaderMaxVectorSize) {
    return nullptr;
  }
  uint8_t* buffer_next = buffer + sizeof(CountT) + sizeof(I) * src_count;
  if (end != nullptr) {
    if (buffer_next > end) return nullptr;
    buffer = WriteToBuffer(buffer, end, static_cast<CountT>(src_count));
    // TODO endianess?
    std::copy(source.begin(), source.end(), reinterpret_cast<I*>(buffer));
  }
  return buffer_next;
}

template <typename I, typename C>
const uint8_t* ReadContainerFromBuffer(const uint8_t* buffer, const uint8_t* end, C& target) {
  static_assert(std::is_same<I, typename C::value_type>::value, "value types mismatch");
  CountT count_raw;
  buffer = ReadFromBuffer(buffer, end, count_raw);
  if (!buffer) return nullptr;
  const size_t count = static_cast<size_t>(count_raw);
  if (count > kHeaderMaxVectorSize) return nullptr;
  const uint8_t* buffer_next = buffer + sizeof(I) * count;
  if (buffer_next > end) return nullptr;
  target.resize(count);
  const I* src = reinterpret_cast<const I*>(buffer);
  // TODO endianess?
  std::copy(src, src + count, target.begin());
  return buffer_next;
}

}  // namespace

template <typename F>
uint8_t* SerializeImpl(const Header& header, bool with_tip, uint32_t* ptr_tip, F resize) {
  size_t length;
  uint32_t magic;
  if (!Size(header, with_tip, &length, &magic)) {
    return nullptr;
  }
  if (ptr_tip) {
    *ptr_tip = magic;
  }
  // Prepare buffer
  uint8_t* ptr = resize(length);
  if (!ptr) {
    return nullptr;
  }
  uint8_t* ptr_end = ptr + length;
  // Write magic
  if (with_tip) {
    ptr = WriteToBuffer(ptr, ptr_end, static_cast<uint32_t>(magic));
    ASSERT(ptr, "logic error: could not write tip");
  }
  // Write data
  if (header.timestamp) {
    ptr = WriteToBuffer(ptr, ptr_end, *header.timestamp);
    ASSERT(ptr, "logic error: could not write timestamp");
  }
  if (header.uuid) {
    ptr = WriteToBuffer(ptr, ptr_end, *header.uuid);
    ASSERT(ptr, "logic error: could not write uuid");
  }
  if (!header.tag.empty()) {
    ptr = WriteContainerToBuffer<char>(ptr, ptr_end, header.tag);
    ASSERT(ptr, "logic error: could not write tag");
  }
  if (header.acqtime) {
    ptr = WriteToBuffer(ptr, ptr_end, *header.acqtime);
    ASSERT(ptr, "logic error: could not write acqtime");
  }
  if (header.format) {
    ptr = WriteToBuffer(ptr, ptr_end, *header.format);
    ASSERT(ptr, "logic error: could not write format");
  }
  if (!header.minipayload.empty()) {
    ptr = WriteContainerToBuffer<uint8_t>(ptr, ptr_end, header.minipayload);
    ASSERT(ptr, "logic error: could not write minipayload");
  }
  if (!header.segments.empty()) {
    ptr = WriteContainerToBuffer<uint32_t>(ptr, ptr_end, header.segments);
    ASSERT(ptr, "logic error: could not write segments");
  }
  if (!header.buffers.empty()) {
    ptr = WriteContainerToBuffer<uint32_t>(ptr, ptr_end, header.buffers);
    ASSERT(ptr, "logic error: could not write buffers");
  }
  if (header.proto_id) {
    const uint64_t value = *header.proto_id;
    ptr = WriteToBuffer<uint64_t>(ptr, ptr_end, value);
    ASSERT(ptr, "logic error: could not write buffers");
  }
  return ptr;
}

const uint8_t* DeserializeImpl(const uint8_t* ptr, const uint8_t* ptr_end, uint32_t* ptr_tip,
                               Header& header) {
  // read magic byte
  uint32_t magic;
  if (ptr_tip) {
    magic = *ptr_tip;
  } else {
    ptr = ReadFromBuffer(ptr, ptr_end, magic);
    if (ptr == nullptr) return nullptr;
  }
  if (magic & TIP_1_TIMESTAMP) {
    int64_t value;
    ptr = ReadFromBuffer(ptr, ptr_end, value);
    if (!ptr) return nullptr;
    header.timestamp = value;
  } else {
    header.timestamp = std::nullopt;
  }
  if (magic & TIP_2_UUID) {
    Uuid value;
    ptr = ReadFromBuffer(ptr, ptr_end, value);
    if (!ptr) return nullptr;
    header.uuid = value;
  } else {
    header.uuid = std::nullopt;
  }
  if (magic & TIP_3_TAG) {
    ptr = ReadContainerFromBuffer<char>(ptr, ptr_end, header.tag);
    if (!ptr) return nullptr;
  } else {
    header.tag.clear();
  }
  if (magic & TIP_4_ACQTIME) {
    int64_t value;
    ptr = ReadFromBuffer(ptr, ptr_end, value);
    if (!ptr) return nullptr;
    header.acqtime = value;
  } else {
    header.acqtime = std::nullopt;
  }
  if (magic & TIP_5_FORMAT) {
    uint64_t value;
    ptr = ReadFromBuffer(ptr, ptr_end, value);
    if (!ptr) return nullptr;
    header.format = value;
  } else {
    header.format = std::nullopt;
  }
  if (magic & TIP_6_MINIPAYLOAD) {
    ptr = ReadContainerFromBuffer<uint8_t>(ptr, ptr_end, header.minipayload);
    if (!ptr) return nullptr;
  } else {
    header.minipayload.clear();
  }
  if (magic & TIP_7_SEGMENTS) {
    ptr = ReadContainerFromBuffer<uint32_t>(ptr, ptr_end, header.segments);
    if (!ptr) return nullptr;
  } else {
    header.segments.clear();
  }
  if (magic & TIP_8_BUFFERS) {
    ptr = ReadContainerFromBuffer<uint32_t>(ptr, ptr_end, header.buffers);
    if (!ptr) return nullptr;
  } else {
    header.buffers.clear();
  }
  if (magic & TIP_9_PROTO_ID) {
    uint64_t value;
    ptr = ReadFromBuffer<uint64_t>(ptr, ptr_end, value);
    if (!ptr) return nullptr;
    header.proto_id = value;
  } else {
    header.proto_id = std::nullopt;
  }
  return ptr;
}

bool Size(const Header& header, bool with_tip, size_t* size, uint32_t* tip) {
  // Compute header
  uint32_t magic = 0;
  uint8_t* ptr = nullptr;
  if (header.timestamp) {
    magic |= TIP_1_TIMESTAMP;
    ptr = WriteToBuffer(ptr, nullptr, *header.timestamp);
    if (!ptr) return false;
  }
  if (header.uuid) {
    magic |= TIP_2_UUID;
    ptr = WriteToBuffer(ptr, nullptr, *header.uuid);
    if (!ptr) return false;
  }
  if (!header.tag.empty()) {
    magic |= TIP_3_TAG;
    ptr = WriteContainerToBuffer<char>(ptr, nullptr, header.tag);
    if (!ptr) return false;
  }
  if (header.acqtime) {
    magic |= TIP_4_ACQTIME;
    ptr = WriteToBuffer(ptr, nullptr, *header.acqtime);
    if (!ptr) return false;
  }
  if (header.format) {
    magic |= TIP_5_FORMAT;
    ptr = WriteToBuffer(ptr, nullptr, *header.format);
    if (!ptr) return false;
  }
  if (!header.minipayload.empty()) {
    magic |= TIP_6_MINIPAYLOAD;
    ptr = WriteContainerToBuffer<uint8_t>(ptr, nullptr, header.minipayload);
    if (!ptr) return false;
  }
  if (!header.segments.empty()) {
    magic |= TIP_7_SEGMENTS;
    ptr = WriteContainerToBuffer<uint32_t>(ptr, nullptr, header.segments);
    if (!ptr) return false;
  }
  if (!header.buffers.empty()) {
    magic |= TIP_8_BUFFERS;
    ptr = WriteContainerToBuffer<uint32_t>(ptr, nullptr, header.buffers);
    if (!ptr) return false;
  }
  if (header.proto_id) {
    magic |= TIP_9_PROTO_ID;
    ptr = WriteToBuffer<uint64_t>(ptr, nullptr, *header.proto_id);
    if (!ptr) return false;
  }
  if (with_tip) {
    ptr = WriteToBuffer(ptr, nullptr, magic);
    if (!ptr) return false;
  }
  const size_t length = std::distance(static_cast<uint8_t*>(nullptr), ptr);
  ASSERT(length <= kHeaderMaxLength, "Header too long: %zu", length);
  if (size) *size = length;
  if (tip) *tip = magic;
  return true;
}

bool Serialize(const Header& header, std::vector<uint8_t>& buffer) {
  return SerializeImpl(header, true, nullptr, [&](size_t n) {
           buffer.resize(n);
           return buffer.data();
         }) != nullptr;
}

uint8_t* Serialize(const Header& header, uint8_t* begin, uint8_t* end) {
  return SerializeImpl(header, true, nullptr, [=](size_t n) -> uint8_t* {
    if (begin + n > end) {
      return nullptr;
    }
    return begin;
  });
}

bool Deserialize(const std::vector<uint8_t>& buffer, Header& header) {
  return DeserializeImpl(buffer.data(), buffer.data() + buffer.size(), nullptr, header) != nullptr;
}

const uint8_t* Deserialize(const uint8_t* begin, const uint8_t* end, Header& header) {
  return DeserializeImpl(begin, end, nullptr, header);
}

bool SerializeWithoutTip(const Header& header, uint32_t magic, std::vector<uint8_t>& buffer) {
  uint32_t magic_actual;
  const bool read_ok = SerializeImpl(header, false, &magic_actual, [&](size_t n) {
                         buffer.resize(n);
                         return buffer.data();
                       }) != nullptr;
  return read_ok && magic == magic_actual;
}

uint8_t* SerializeWithoutTip(const Header& header, uint32_t magic, uint8_t* begin, uint8_t* end) {
  uint32_t magic_actual;
  uint8_t* ptr = SerializeImpl(header, false, &magic_actual, [=](size_t n) -> uint8_t* {
    if (begin + n > end) {
      return nullptr;
    }
    return begin;
  });
  if (!ptr) return nullptr;
  if (magic != magic_actual) return nullptr;
  return ptr;
}

bool DeserializeWithoutTip(const std::vector<uint8_t>& buffer, uint32_t magic, Header& header) {
  return DeserializeImpl(buffer.data(), buffer.data() + buffer.size(), &magic, header);
}

const uint8_t* DeserializeWithoutTip(const uint8_t* begin, const uint8_t* end, uint32_t magic,
                                     Header& header) {
  return DeserializeImpl(begin, end, &magic, header);
}

}  // namespace serialization
}  // namespace isaac
