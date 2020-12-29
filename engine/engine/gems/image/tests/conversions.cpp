/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/image/conversions.hpp"

#include "engine/core/image/image.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

void CheckPixelEq(const Pixel3f& p1, const Pixel3f& p2) {
  EXPECT_FLOAT_EQ(p1[0], p2[0]);
  EXPECT_FLOAT_EQ(p1[1], p2[1]);
  EXPECT_FLOAT_EQ(p1[2], p2[2]);
}

TEST(color_conversion, ConvertYuyvToRgb) {
  Image2ub yuyv(2, 2);
  Image3ub rgb;

  // First row
  yuyv(0, 0) = Pixel2ub{50, 100};   // Y, Cb channels
  yuyv(0, 1) = Pixel2ub{150, 200};  // Y, Cr channels

  // Second row
  yuyv(1, 0) = Pixel2ub{16, 128};  // Y, Cb channels
  yuyv(1, 1) = Pixel2ub{75, 128};  // Y, Cr channels

  ConvertYuyvToRgb(yuyv, rgb);
  ASSERT_EQ(rgb.rows(), yuyv.rows());
  ASSERT_EQ(rgb.cols(), yuyv.cols());

  // Pixel (0, 0)
  ASSERT_EQ(rgb(0, 0)[0], 154);
  ASSERT_EQ(rgb(0, 0)[1], 0);
  ASSERT_EQ(rgb(0, 0)[2], 0);

  // Pixel (0, 1)
  ASSERT_EQ(rgb(0, 1)[0], 255);
  ASSERT_EQ(rgb(0, 1)[1], 108);
  ASSERT_EQ(rgb(0, 1)[2], 99);

  // Pixel (1, 1)
  ASSERT_EQ(rgb(1, 1)[0], 68);
  ASSERT_EQ(rgb(1, 1)[1], 68);
  ASSERT_EQ(rgb(1, 1)[2], 68);

  // Pixel (1, 0)
  ASSERT_EQ(rgb(1, 0)[0], 0);
  ASSERT_EQ(rgb(1, 0)[1], 0);
  ASSERT_EQ(rgb(1, 0)[2], 0);
}

TEST(RgbToHsv, black) {
  const Pixel3f rgb{0.0f, 0.0f, 0.0f};
  const Pixel3f hsv{0.0f, 0.0f, 0.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, white) {
  const Pixel3f rgb{1.0f, 1.0f, 1.0f};
  const Pixel3f hsv{0.0f, 0.0f, 255.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, red) {
  const Pixel3f rgb{1.0f, 0.0f, 0.0f};
  const Pixel3f hsv{0.0f, 255.0f, 255.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, green) {
  const Pixel3f rgb{0.0f, 1.0f, 0.0f};
  const Pixel3f hsv{120.0f, 255.0f, 255.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, blue) {
  const Pixel3f rgb{0.0f, 0.0f, 1.0f};
  const Pixel3f hsv{240.0f, 255.0f, 255.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, yellow) {
  const Pixel3f rgb{1.0f, 1.0f, 0.0f};
  const Pixel3f hsv{60.0f, 255.0f, 255.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, magenta) {
  const Pixel3f rgb{1.0f, 0.0f, 1.0f};
  const Pixel3f hsv{300.0f, 255.0f, 255.0f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, gray) {
  const Pixel3f rgb{0.5f, 0.5f, 0.5f};
  const Pixel3f hsv{0.0f, 0.0f, 127.5f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(RgbToHsv, purple) {
  const Pixel3f rgb{0.5f, 0.0f, 0.5f};
  const Pixel3f hsv{300.0f, 255.0f, 127.5f};
  CheckPixelEq(RgbToHsv(rgb), hsv);
}

TEST(tensor_encoder, ImageToNormalizedTensor) {
  Image3ub rgb_image(2, 2);
  rgb_image(0, 0) = Pixel3ub{0, 0, 0};
  rgb_image(0, 1) = Pixel3ub{0, 255, 0};
  rgb_image(1, 1) = Pixel3ub{255, 255, 255};
  rgb_image(1, 0) = Pixel3ub{100, 200, 50};

  Tensor3f tensor(2, 2, 3);
  ImageToNormalizedTensor(rgb_image, tensor.view());
  EXPECT_EQ(tensor.element_count(), rgb_image.num_elements());

  // Pixel (0, 0)
  EXPECT_FLOAT_EQ(tensor(0, 0, 0), -1);  // red channel
  EXPECT_FLOAT_EQ(tensor(0, 0, 1), -1);  // green channel
  EXPECT_FLOAT_EQ(tensor(0, 0, 2), -1);  // blue channel

  // Pixel (0, 1)
  EXPECT_FLOAT_EQ(tensor(0, 1, 0), -1);  // red channel
  EXPECT_FLOAT_EQ(tensor(0, 1, 1), 1);   // green channel
  EXPECT_FLOAT_EQ(tensor(0, 1, 2), -1);  // blue channel

  // Pixel (1, 1)
  EXPECT_FLOAT_EQ(tensor(1, 1, 0), 1);  // red channel
  EXPECT_FLOAT_EQ(tensor(1, 1, 1), 1);  // green channel
  EXPECT_FLOAT_EQ(tensor(1, 1, 2), 1);  // blue channel

  // Pixel (1, 0)
  EXPECT_FLOAT_EQ(tensor(1, 0, 0), 100.0f / 127.5f - 1.0f);  // red channel
  EXPECT_FLOAT_EQ(tensor(1, 0, 1), 200.0f / 127.5f - 1.0f);  // green channel
  EXPECT_FLOAT_EQ(tensor(1, 0, 2), 50.0f / 127.5f - 1.0f);   // blue channel
}

TEST(tensor_encoder, SingleChannelImageToNormalizedTensor) {
  Image1ub label_image(2, 2);
  label_image(0, 0) = Pixel1ub{0};
  label_image(0, 1) = Pixel1ub{255};
  label_image(1, 1) = Pixel1ub{100};
  label_image(1, 0) = Pixel1ub{50};

  Tensor2f tensor(2, 2);
  ImageToNormalizedTensor(label_image, tensor.view());
  EXPECT_EQ(tensor.element_count(), label_image.num_elements());

  // Pixel (0, 0)
  EXPECT_FLOAT_EQ(tensor(0, 0), -1);

  // Pixel (0, 1)
  EXPECT_FLOAT_EQ(tensor(0, 1), 1);

  // Pixel (1, 1)
  EXPECT_FLOAT_EQ(tensor(1, 1), 100.0f / 127.5f - 1.0f);

  // Pixel (1, 0)
  EXPECT_FLOAT_EQ(tensor(1, 0), 50.0f / 127.5f - 1.0f);
}

TEST(Conversions, ConvertRgba4fToRgb) {
  Image4f source(20, 30);
  FillPixels(source, Pixel4f{0.1, 1.0, 0.5, 0.7});
  Image3ub actual(20, 30);
  ConvertRgbaToRgb(source, actual);
  for (int row = 0; row < actual.rows(); row++) {
    for (int col = 0; col < actual.cols(); col++) {
      const auto pxl = actual(row, col);
      ASSERT_EQ(pxl[0], 26);
      ASSERT_EQ(pxl[1], 255);
      ASSERT_EQ(pxl[2], 128);
    }
  }
}

TEST(Conversions, ConvertRgba3ubToRgb) {
  Image4ub source(20, 30);
  FillPixels(source, Pixel4ub{51, 118, 183, 35});
  Image3ub actual(20, 30);
  ConvertRgbaToRgb(source, actual);
  for (int row = 0; row < actual.rows(); row++) {
    for (int col = 0; col < actual.cols(); col++) {
      const auto pxl = actual(row, col);
      ASSERT_EQ(pxl[0], 51);
      ASSERT_EQ(pxl[1], 118);
      ASSERT_EQ(pxl[2], 183);
    }
  }
}

TEST(Conversions, ConvertBgraToRgb) {
  Image4ub source(20, 30);
  FillPixels(source, Pixel4ub{54, 117, 187, 37});
  Image3ub actual(20, 30);
  ConvertBgraToRgb(source, actual);
  for (int row = 0; row < actual.rows(); row++) {
    for (int col = 0; col < actual.cols(); col++) {
      const auto pxl = actual(row, col);
      ASSERT_EQ(pxl[0], 187);
      ASSERT_EQ(pxl[1], 117);
      ASSERT_EQ(pxl[2], 54);
    }
  }
}

TEST(Conversions, ConvertRgbToRgba) {
  Image3ub source(20, 30);
  FillPixels(source, Pixel3ub{59, 112, 184});
  Image4ub actual(20, 30);
  ConvertRgbToRgba(source, actual, 99);
  for (int row = 0; row < actual.rows(); row++) {
    for (int col = 0; col < actual.cols(); col++) {
      const auto pxl = actual(row, col);
      ASSERT_EQ(pxl[0], 59);
      ASSERT_EQ(pxl[1], 112);
      ASSERT_EQ(pxl[2], 184);
      ASSERT_EQ(pxl[3], 99);
    }
  }
}

TEST(Conversions, CpuVsGpuRgbImageToTensor) {
  Image3ub image;
  LoadPng("engine/gems/image/data/left.png", image);

  for (const auto normalization : {ImageToTensorNormalization::kUnit,
                                   ImageToTensorNormalization::kPositiveNegative,
                                   ImageToTensorNormalization::kNone,
                                   ImageToTensorNormalization::kHalfAndHalf}) {
    const Vector3i result_dimensions(image.rows(), image.cols(), 3);

    // Convert on CPU
    Tensor3f cpu_result(result_dimensions);
    ImageToNormalizedTensor(image, cpu_result.view(), normalization);

    // Convert on GPU
    CudaImage3ub cuda_image(image.dimensions());
    Copy(image, cuda_image);
    CudaTensor3f cuda_result(result_dimensions);
    ImageToNormalizedTensor(cuda_image.const_view(), cuda_result.view(), normalization);
    Tensor3f cuda_result_copy(result_dimensions);
    Copy(cuda_result, cuda_result_copy);

    // Check that both methods gave the same result.
    for (int i = 0; i < result_dimensions[0]; ++i) {
      for (int j = 0; j < result_dimensions[1]; ++j) {
        for (int k = 0; k < result_dimensions[2]; ++k) {
          ASSERT_NEAR(cpu_result(i, j, k), cuda_result_copy(i, j, k), 1e-6f);
        }
      }
    }
  }
}

TEST(Conversions, CpuVsGpuSingleChannelImageToTensor) {
  Image1ub image;
  LoadPng("engine/gems/image/data/label.png", image);

  for (const auto normalization : {ImageToTensorNormalization::kUnit,
                                   ImageToTensorNormalization::kPositiveNegative,
                                   ImageToTensorNormalization::kNone,
                                   ImageToTensorNormalization::kHalfAndHalf}) {
    const auto result_dimensions = Tensor2f::dimensions_t{image.rows(), image.cols()};
    // Convert on CPU
    Tensor2f cpu_result(result_dimensions);
    ImageToNormalizedTensor(image, cpu_result.view(), normalization);
    // Convert on GPU
    CudaImage1ub cuda_image(image.dimensions());
    Copy(image, cuda_image);
    CudaTensor2f cuda_result(result_dimensions);
    ImageToNormalizedTensor(cuda_image.const_view(), cuda_result.view(), normalization);
    Tensor2f cuda_result_copy(cuda_result.dimensions());
    Copy(cuda_result, cuda_result_copy);
    // Check that both methods gave the same result.
    for (int i = 0; i < 1; ++i) {
      for (int j = 0; j < 1; ++j) {
          ASSERT_NEAR(cpu_result(i, j), cuda_result_copy(i, j), 1e-6f);
      }
    }
  }
}

}  // namespace isaac