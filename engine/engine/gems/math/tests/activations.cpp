/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/math/activations.hpp"

#include "gtest/gtest.h"
namespace isaac {

// Test standard sigmoid equation with different inputs
TEST(Activations, Sigmoid) {
  EXPECT_EQ(Sigmoid(0.0f), 0.5f);
  EXPECT_NEAR(Sigmoid(1.0f), 0.731f, 0.001f);
  EXPECT_NEAR(Sigmoid(-1.0f), 0.268f, 0.001f);
}
}  // namespace isaac
