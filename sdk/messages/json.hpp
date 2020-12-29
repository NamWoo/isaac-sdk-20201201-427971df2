/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json_formatter.hpp"
#include "messages/json.capnp.h"

namespace isaac {

inline std::optional<Json> FromProto(::JsonProto::Reader reader) {
  return serialization::ParseJson(reader.getSerialized());
}

inline void ToProto(const Json& json, ::JsonProto::Builder builder) {
  builder.setSerialized(json.dump());
}

}  // namespace isaac
