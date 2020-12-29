/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
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

TEST(State, Accessors) {
  FooBarTur actual;
  actual.foo() = kFoo;
  actual.bar() = kBar;
  actual.tur() = kTur;
  EXPECT_EQ(actual.foo(), kFoo);
  EXPECT_EQ(actual.bar(), kBar);
  EXPECT_EQ(actual.tur(), kTur);
}

TEST(State, Indices) {
  ASSERT_GE(FooBarTur::kI_foo, 0);
  ASSERT_LT(FooBarTur::kI_foo, 3);
  ASSERT_GE(FooBarTur::kI_bar, 0);
  ASSERT_LT(FooBarTur::kI_bar, 3);
  ASSERT_GE(FooBarTur::kI_tur, 0);
  ASSERT_LT(FooBarTur::kI_tur, 3);
  FooBarTur actual;
  actual.elements[FooBarTur::kI_foo] = kFoo;
  actual.elements[FooBarTur::kI_bar] = kBar;
  actual.elements[FooBarTur::kI_tur] = kTur;
  EXPECT_EQ(actual.foo(), kFoo);
  EXPECT_EQ(actual.bar(), kBar);
  EXPECT_EQ(actual.tur(), kTur);
}

}  // namespace state
}  // namespace isaac
