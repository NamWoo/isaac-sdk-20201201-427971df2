/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "messages/geometry.hpp"

#include <vector>

#include "capnp/message.h"
#include "gtest/gtest.h"

namespace isaac {

TEST(Geometry, Polyline2dProtoRoundtrip) {
  geometry::Polyline2d inPolyline;
  {
    inPolyline.resize(rand() % 256);
    for (uint32_t i = 0; i < inPolyline.size(); i++) {
      inPolyline[i] = {rand() % 256, rand() % 256};
    }
  }

  ::capnp::MallocMessageBuilder message;
  ::Polyline2dProto::Builder builder = message.initRoot<::Polyline2dProto>();
  ToProto(inPolyline, builder);

  geometry::Polyline2d outPolyline = FromProto(message.getRoot<::Polyline2dProto>());
  {
    ASSERT_EQ(outPolyline.size(), inPolyline.size());

    for (uint32_t i = 0; i < inPolyline.size(); i++) {
      ASSERT_EQ(inPolyline[i], outPolyline[i]);
    }
  }
}

}  // namespace isaac
