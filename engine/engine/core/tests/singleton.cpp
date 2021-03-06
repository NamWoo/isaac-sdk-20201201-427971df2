/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/singleton.hpp"

#include <iostream>

#include "gtest/gtest.h"

namespace isaac {

int outside = 0;

struct Foo {
  Foo() {
    outside = 1;
    status = 1;
  }
  int status = 0;
};

TEST(Core, Stacktrace) {
  EXPECT_EQ(outside, 1);
  EXPECT_EQ(Singleton<Foo>::Get().status, 1);
}

}  // namespace isaac
