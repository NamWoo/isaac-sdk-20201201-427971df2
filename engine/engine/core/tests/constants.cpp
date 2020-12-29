/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/constants.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace isaac {

TEST(DegRadConversion, Identity) {
  for (double value = -200.0; value < 200.0; value += 0.1) {
    EXPECT_NEAR(value, DegToRad(RadToDeg(value)), 1e-7);
    EXPECT_NEAR(value, RadToDeg(DegToRad(value)), 1e-7);
  }
}

TEST(DegRadConversion, RadToDeg) {
  EXPECT_NEAR(0.0, RadToDeg(0.0), 1e-7);
  EXPECT_NEAR(180.0, RadToDeg(M_PI), 1e-7);
  EXPECT_NEAR(-180.0, RadToDeg(-M_PI), 1e-7);
  EXPECT_NEAR(90.0, RadToDeg(M_PI * 0.5), 1e-7);
  EXPECT_NEAR(-90.0, RadToDeg(-M_PI * 0.5), 1e-7);
}

TEST(DegRadConversion, DegToRad) {
  EXPECT_NEAR(DegToRad(0.0), 0.0, 1e-7);
  EXPECT_NEAR(DegToRad(180.0), M_PI, 1e-7);
  EXPECT_NEAR(DegToRad(-180.0), -M_PI, 1e-7);
  EXPECT_NEAR(DegToRad(90.0), M_PI * 0.5, 1e-7);
  EXPECT_NEAR(DegToRad(-90.0), -M_PI * 0.5, 1e-7);
}

TEST(Pi, pi_twopi) {
  EXPECT_NEAR(Pi<double> + Pi<double>, TwoPi<double>, 1e-15);
  EXPECT_NEAR(Pi<float> + Pi<float>, TwoPi<float>, 1e-7);
}

}  // namespace isaac
