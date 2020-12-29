/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/types.hpp"
#include "gtest/gtest.h"

#define ISAAC_EXPECT_VEC_EQ(A, B) \
  { \
    auto _a = (A); \
    auto _b = (B); \
    ASSERT_EQ(_a.size(), _b.size()); \
    const size_t a_length = _a.size(); \
    for (size_t i = 0; i < a_length; i++) { \
      EXPECT_EQ(_a[i], _b[i]); \
    } \
  }

#define ISAAC_EXPECT_VEC_NEAR(A, B, T) \
  { \
    auto _a = (A); \
    auto _b = (B); \
    ASSERT_EQ(_a.size(), _b.size()); \
    const size_t a_length = _a.size(); \
    for (size_t i = 0; i < a_length; i++) { \
      EXPECT_NEAR(_a[i], _b[i], T); \
    } \
  }

#define ISAAC_EXPECT_MAT_NEAR(A, B, T) \
  { \
    auto _a = (A); \
    auto _b = (B); \
    ASSERT_EQ(_a.rows(), _b.rows()); \
    ASSERT_EQ(_a.cols(), _b.cols()); \
    for (int i = 0; i < _a.rows(); i++) { \
      for (int j = 0; j < _a.cols(); j++) { \
        EXPECT_NEAR(_a(i, j), _b(i, j), T); \
      } \
    } \
  }

#define ISAAC_ASSERT_VEC_NEAR(A, B, T) \
  { \
    auto _a = (A); \
    auto _b = (B); \
    ASSERT_EQ(_a.size(), _b.size()); \
    const size_t a_length = _a.size(); \
    for (size_t i = 0; i < a_length; i++) { \
      ASSERT_NEAR(_a[i], _b[i], T); \
    } \
  }

#define ISAAC_EXPECT_VEC_NEAR_ZERO(A, T) \
  { \
    auto _a = (A); \
    const size_t a_length = _a.size(); \
    for (size_t i = 0; i < a_length; i++) { \
      EXPECT_NEAR(_a[i], 0, T); \
    } \
  }

#define ISAAC_EXPECT_SO_NEAR_ID(A, T) \
  EXPECT_NEAR((A).angle(), 0.0, T);

#define ISAAC_EXPECT_POSE_NEAR_ID(A, T) \
  { \
    ISAAC_EXPECT_SO_NEAR_ID((A).rotation, T); \
    ISAAC_EXPECT_VEC_NEAR_ZERO((A).translation, T); \
  }

#define ISAAC_EXPECT_SO_NEAR(A, B, T) \
  ISAAC_EXPECT_SO_NEAR_ID((A) * (B).inverse(), T);

#define ISAAC_EXPECT_POSE_NEAR(A, B, T) \
  ISAAC_EXPECT_POSE_NEAR_ID((A) * (B).inverse(), T);
