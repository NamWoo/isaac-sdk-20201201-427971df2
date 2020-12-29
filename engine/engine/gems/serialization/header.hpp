/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "engine/core/optional.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace serialization {

// At the start of the header is a bitmask which indicates which fields of the header are set.
// The options are available through the TipBits enum.
enum TipBits {
  TIP_1_TIMESTAMP = 1 << 0,
  TIP_2_UUID = 1 << 1,
  TIP_3_TAG = 1 << 2,
  TIP_4_ACQTIME = 1 << 3,
  TIP_5_FORMAT = 1 << 4,
  TIP_6_MINIPAYLOAD = 1 << 5,
  TIP_7_SEGMENTS = 1 << 6,
  TIP_8_BUFFERS = 1 << 7,
  TIP_9_PROTO_ID = 1 << 8
};

// A header for messages or other chunks of data. Every data member is optional. Integral types
// are using std::optional and can be set to std::nullopt if they are not used. List types
// and std::strings would are not used in case they are empty.
struct Header {
  // A 64-bit timestamp. If used if will take 8 bytes.
  std::optional<int64_t> timestamp;
  // A unique identifier. If stored it will take 16 bytes.
  std::optional<Uuid> uuid;
  // A string for example to tag the message as part of a group. If stored it will take
  // 1 + size bytes.
  std::string tag;
  // A second 64-bit timestamp. If stored it will take 8 bytes.
  std::optional<int64_t> acqtime;
  // An index indicating the format of the message
  std::optional<uint64_t> format;
  // A small data blob which can contain arbitrary data. The maximum size is 256 bytes. If used it
  // will take 1 + size bytes.
  std::vector<uint8_t> minipayload;
  // A list of 32 bit integers, for example to indicate dimensions of blobs attached to the
  // message. The maximun length is 256 elements. If used it will take 1 + 4*size bytes.
  std::vector<uint32_t> segments;
  // A list of 32-bit integers for example to indicate dimensions of large blobs attached to the
  // message. The maximun length is 256 elements. If used it will take 1 + 4*size bytes.
  std::vector<uint32_t> buffers;
  // A 64-bit id of proto stored retrieved via ::capnp::typeId<Proto>()
  std::optional<uint64_t> proto_id;
};

// Maximum number of elements in the header members tag, minipayload, segments or buffers
static constexpr size_t kHeaderMaxVectorSize = 255;
// Maximum length of a serialized header
static constexpr size_t kHeaderMaxLength = 2598;

// Computes the serialized size and the tip for a header
bool Size(const Header& header, bool with_tip, size_t* length, uint32_t* tip);

// Serializes a header to a buffer. If the given vector is too small it will be resized. Returns
// false if an error occurred while serializing the header.
bool Serialize(const Header& header, std::vector<uint8_t>& buffer);
// Serializes a header to a buffer. The returned pointer will point to the first byte after the
// header. If the given range is too small or an error occurred while serializing the header this
// function will return nullptr.
uint8_t* Serialize(const Header& header, uint8_t* begin, uint8_t* end);
// Deserializes a header from a buffer or a range. Returns false if an error occurs while
// deserializing the header.
bool Deserialize(const std::vector<uint8_t>& buffer, Header& header);
const uint8_t* Deserialize(const uint8_t* begin, const uint8_t* end, Header& header);

// Similar to `Serialize`, but does not write the first 4 bytes for the tip which contains the
// structure of the header. Also returns false if the given tip does not match the header structure.
bool SerializeWithoutTip(const Header& header, uint32_t tip, std::vector<uint8_t>& buffer);
uint8_t* SerializeWithoutTip(const Header& header, uint32_t tip, uint8_t* begin, uint8_t* end);

// Similar to `Deserialize`, but does not read the first 4 bytes for the tip which contains the
// structure of the header.
bool DeserializeWithoutTip(const std::vector<uint8_t>& buffer, uint32_t tip, Header& header);
const uint8_t* DeserializeWithoutTip(const uint8_t* begin, const uint8_t* end, uint32_t tip,
                                     Header& header);

}  // namespace serialization
}  // namespace isaac
