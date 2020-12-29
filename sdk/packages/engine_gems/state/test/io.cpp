/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/engine_gems/state/io.hpp"

#include "capnp/message.h"
#include "engine/core/buffers/shared_buffer.hpp"
#include "packages/engine_gems/state/state.hpp"
#include "packages/engine_gems/state/test/domains.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace state {

namespace {
constexpr double kFoo = 13.1;
constexpr double kBar = -0.3;
constexpr double kTur = 0.57;
}  // namespace

TEST(State, ToFromProto) {
  FooBarTur actual;
  actual.foo() = kFoo;
  actual.bar() = kBar;
  actual.tur() = kTur;
  ::capnp::MallocMessageBuilder message;
  auto builder = message.initRoot<StateProto>();
  std::vector<isaac::SharedBuffer> buffers;
  ToProto(actual, builder, buffers);

  auto reader = builder.asReader();
  FooBarTur expected;
  FromProto(reader, buffers, expected);
  EXPECT_EQ(expected.foo(), kFoo);
  EXPECT_EQ(expected.bar(), kBar);
  EXPECT_EQ(expected.tur(), kTur);
}

}  // namespace state
}  // namespace isaac
