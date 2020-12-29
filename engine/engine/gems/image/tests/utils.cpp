/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/image/utils.hpp"

#include <random>

#include "engine/core/image/image.hpp"
#include "engine/gems/image/io.hpp"
#include "gtest/gtest.h"

namespace isaac {

namespace {

// Helper function for `ShiftImageInplace` test
void ShiftImageInplaceTest(int dx, int dy) {
  const int rows = 30;
  const int cols = 40;
  Image1f test(rows, cols);
  FillPixels(test, 1.0f);
  ShiftImageInplace(dx, dy, 0.0f, test);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      const bool oldcell = test.isValidCoordinate(row + dx, col + dy);
      const float expected = oldcell ? 1.0f : 0.0f;
      ASSERT_EQ(test(row, col), expected)
          << "\n\tdx: " << dx  << "\n\tdy: " << dy
          << "\n\tcol: " << col  << "\n\trow: " << row;
    }
  }
}

}

TEST(images, enlarge_reduce) {
  Image1ub gt;
  ASSERT_TRUE(LoadPng("engine/gems/image/data/room.png", gt));

  Image1ub big = Enlarge<3>(gt);
  ASSERT_EQ(big.rows(), 3 * gt.rows());
  ASSERT_EQ(big.cols(), 3 * gt.cols());

  Image1ub img = Reduce<3>(big);
  ASSERT_EQ(img.rows(), gt.rows());
  ASSERT_EQ(img.cols(), gt.cols());

  ASSERT_EQ(img.num_pixels(), gt.num_pixels());
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    ASSERT_EQ(gt[pixel], img[pixel]);
  }
}

TEST(Utils, Convert) {
  Image1f test(30, 40);
  FillPixels(test, 0.5f);

  auto f = [](float x) { return 17; };

  Image1ub result1 = Convert<Image1ub>(test, f);
  EXPECT_EQ(result1.dimensions(), test.dimensions());

  Image1ub result2;
  EXPECT_DEATH(Convert(test, result2, f), ".?dimensions mismatch.?");

  Image1ub result3(test.dimensions());
  ImageView1ub view3 = result3;
  Convert(test, view3, f);
  EXPECT_EQ(result3.dimensions(), test.dimensions());
}

TEST(Utils, ShiftImageInplace) {
  ShiftImageInplaceTest(2, 4);
  ShiftImageInplaceTest(2, -4);
  ShiftImageInplaceTest(-2, 4);
  ShiftImageInplaceTest(-2, -4);
}

}  // namespace isaac