/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/core/epsilon.hpp"
#include "engine/gems/math/icosphere_sampling.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "engine/core/math/utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(IcosphereSample, UnitIcosahedron) {
  std::vector<Vector3d> sample = IcosphereSample<double>(12, 1.0, {0.0, 0.0, 0.0});
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 12);
  // check norm of returned vertices
  for (const auto pt: sample) {
    ASSERT_NEAR(pt.norm(), 1.0, 1e-9);
  }
  // check spread of returned vertices
  double projection = 0.44722;
  for (int i=0; i<11; i++) {
    for (int j = i + 1; j < 12; j++) {
      ASSERT_LE(sample[i].dot(sample[j]), projection);
    }
  }
}

TEST(IcosphereSample, RadiusIcosahedron) {
  double radius = 3.6;
  std::vector<Vector3d> sample = IcosphereSample<double>(9, radius, {0.0, 0.0, 0.0});
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 12);
  // check norm of returned vertices
  for (const auto pt: sample) {
    ASSERT_NEAR(pt.norm(), radius, 1e-9);
  }
}

TEST(IcosphereSample, CenterIcosahedron) {
  double radius = 3.6;
  Vector3d center = Vector3d(1.0, 2.0, 3.0);
  std::vector<Vector3d> sample = IcosphereSample<double>(9, radius, center);
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 12);
  // check norm of returned vertices
  for (const auto pt: sample) {
    ASSERT_NEAR((pt - center).norm(), radius, 1e-9);
  }
}

TEST(IcosphereSample, Subdivision) {
  int N = 42;
  double radius = 0.6;
  std::vector<Vector3d> sample = IcosphereSample<double>(N, radius, {0.0, 0.0, 0.0});
  // check number of returned vertices
  ASSERT_EQ(sample.size(), N);
  for (const auto pt: sample) {
    ASSERT_NEAR(pt.norm(), radius, 1e-9);
  }

  N = 41;
  sample = IcosphereSample<double>(N, radius, {0.0, 0.0, 0.0});
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 42);

  N = 43;
  sample = IcosphereSample<double>(N, radius, {0.0, 0.0, 0.0});
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 162);

  N = 2562;
  sample = IcosphereSample<double>(N, radius, {0.0, 0.0, 0.0});
  // check number of returned vertices
  ASSERT_EQ(sample.size(), N);
}

TEST(IcosphereSample, Sorted) {
  int N = 10;
  double radius = 1.0;
  std::vector<Vector3d> sample = IcosphereSample<double>(N, radius, {0.0, 0.0, 0.0}, true);
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 12);
  for (const auto pt: sample) {
    ASSERT_NEAR(pt.norm(), radius, 1e-9);
  }
  // confirm list sorted by z
  for (size_t i = 0; i < sample.size()-1; i++) {
    ASSERT_LE(sample[i][2], sample[i+1][2]);
  }
  // check spread of returned vertices
  double projection = 0.44722;
  for (int i=0; i<11; i++) {
    for (int j = i + 1; j < 12; j++) {
      ASSERT_LE(sample[i].dot(sample[j]), projection);
    }
  }

  N = 20;
  radius = 2.5;
  sample = IcosphereSample<double>(N, radius, {0.0, 0.0, 0.0}, true);
  // check number of returned vertices
  ASSERT_EQ(sample.size(), 42);
  for (const auto pt: sample) {
    ASSERT_NEAR(pt.norm(), radius, 1e-9);
  }
  // confirm list sorted by z
  for (size_t i = 0; i < sample.size()-1; i++) {
    ASSERT_LE(sample[i][2], sample[i+1][2]);
  }
}

}  // namespace isaac
