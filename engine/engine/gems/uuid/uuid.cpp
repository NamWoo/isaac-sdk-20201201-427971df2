/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "uuid.hpp"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <string>

#include "engine/core/assert.hpp"
#include "uuid.h"  // TODO libuuid should have a subfolder // NOLINT

namespace isaac {

Uuid Uuid::Generate() {
  // TODO This mutex might not be necessary if uuid_generate_time is "readonly"
  static std::mutex s_mutex;
  std::lock_guard<std::mutex> lock(s_mutex);
  // We enforce that subsequent calls return different UUIDs. I don't like this, but don't have
  // a better solution for now.
  // FIXME Note that different processes could still generate the same UUID!
  static Uuid s_last_uuid;
  Uuid uuid;
  do {
    uuid_generate_time(uuid.begin());
  } while (uuid == s_last_uuid);
  s_last_uuid = uuid;
  uuid.is_uuid_ = true;
  return uuid;
}

Uuid Uuid::FromString(const std::string& str) {
  Uuid uuid;
  const int result = uuid_parse(const_cast<char*>(str.data()), uuid.bytes_);
  if (result != 0) {
    return FromAsciiString(str);
  } else {
    uuid.is_uuid_ = true;
    return uuid;
  }
}

Uuid Uuid::FromAsciiString(const std::string& str) {
  ASSERT(str.size() <= kNumBytes, "String too long: %zd !<= %zd", str.size(), kNumBytes);
  Uuid uuid;
  std::copy(str.begin(), str.end(), uuid.bytes_);
  std::fill(uuid.bytes_ + str.size(), uuid.bytes_ + kNumBytes, 0);
  uuid.is_uuid_ = false;
  return uuid;
}

Uuid Uuid::FromUuidString(const std::string& str) {
  Uuid uuid;
  const int result = uuid_parse(const_cast<char*>(str.data()), uuid.bytes_);
  ASSERT(result == 0, "'%s' is not a valid UUID", str.c_str());
  uuid.is_uuid_ = true;
  return uuid;
}

void Uuid::unparse() const {
  if (is_uuid_) {
    // FIXME Makre sure this is actually thread-safe
    str_.resize(36);
    // TODO This is a bit dicy as uuid_unparse_lower writes 37 bytes (36 bytes + nullterminator),
    // but we seem to be guaranteed in C++11 that std::string includes a nullterminator.
    uuid_unparse_lower(bytes_, const_cast<char*>(str_.data()));
  } else {
    str_ = std::string(reinterpret_cast<const char*>(bytes_));
  }
}

}  // namespace isaac
