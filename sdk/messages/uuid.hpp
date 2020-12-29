/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/gems/uuid/uuid.hpp"
#include "messages/uuid.capnp.h"

namespace isaac {
namespace alice {

// Writes a UUID to a proto
inline void ToProto(const Uuid& uuid, ::UuidProto::Builder builder) {
  builder.setLower(uuid.lower());
  builder.setUpper(uuid.upper());
}
// Reads a UUID from a proto
inline Uuid FromProto(::UuidProto::Reader reader) {
  return Uuid::FromUInt64(reader.getLower(), reader.getUpper());
}

}  // namespace alice
}  // namespace isaac
