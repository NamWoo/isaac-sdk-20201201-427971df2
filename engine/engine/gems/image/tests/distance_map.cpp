/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/image/distance_map.hpp"

#include "engine/core/image/image.hpp"
#include "gtest/gtest.h"

namespace isaac {

// Build a map for testing.
static Image1ub GetMap() {
  Image1ub img(1024, 1024);
  FillPixels(img, uint8_t{255});
  for (int i = 0; i < 50; i++) {
    img((13 + 29 * i) % img.rows(), (17 + 131 * i) % img.cols()) = 0;
  }
  for (int col = 53; col < 122; col++) {
    img(52, col) = 0;
  }
  for (int row = 13; row < 222; row++) {
    img(row, 80) = 0;
  }

  for (int idx = 200; idx < 400; idx++) {
    img(250, idx) = 0;
    img(idx, 250) = 0;
    img(idx, 350) = 0;
    img(350, idx) = 0;
  }
  return img;
}
static Image1ub img = GetMap();
static Image1d gt = DistanceMap(img, 0, 0.1);

TEST(images, distance_approximated) {
  Image1d dist = QuickDistanceMapApproximated(img, 0, 0.1);
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    ASSERT_GE(gt[pixel] * 1.12, dist[pixel]);
  }
}

TEST(images, quick_distance) {
  Image1d dist = QuickDistanceMap(img, 0, 0.1);
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    ASSERT_GE(gt[pixel] * 1.03, dist[pixel]);
  }
}

// Error of less than 1mm
TEST(images, highres_quick_distance) {
  Image1d dist = QuickDistanceMap<100>(img, 0, 0.1);
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    ASSERT_NEAR(gt[pixel], dist[pixel], 1e-3);
  }
}

// No more error for a very tiny time loss
TEST(images, superhighres_quick_distance) {
  Image1d dist = QuickDistanceMap<1000>(img, 0, 0.1);
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    ASSERT_NEAR(gt[pixel], dist[pixel], 1e-10);
  }
}

}  // namespace