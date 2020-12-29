/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/gems/geometry/pinhole.hpp"
#include "messages/camera.capnp.h"
#include "messages/image.hpp"
#include "messages/math.hpp"

namespace isaac {

// Reads a pinhole model from PinholeProto
inline geometry::PinholeD FromProto(::PinholeProto::Reader reader) {
  geometry::PinholeD pinhole;
  pinhole.dimensions = {reader.getRows(), reader.getCols()};
  pinhole.focal = FromProto(reader.getFocal());
  pinhole.center = FromProto(reader.getCenter());
  return pinhole;
}

// Writes a pinhole model to PinholeProto
inline void ToProto(const geometry::PinholeD& pinhole, ::PinholeProto::Builder builder) {
  builder.setRows(pinhole.dimensions[0]);
  builder.setCols(pinhole.dimensions[1]);
  ToProto(pinhole.focal, builder.initFocal());
  ToProto(pinhole.center, builder.initCenter());
}

// Tries to read a pinhole model from CameraIntrinsicsProto
inline bool FromProto(::CameraIntrinsicsProto::Reader reader, geometry::PinholeD& pinhole) {
  if (!reader.hasPinhole()) {
    return false;
  }
  pinhole = FromProto(reader.getPinhole());
  return true;
}

}  // namespace isaac
