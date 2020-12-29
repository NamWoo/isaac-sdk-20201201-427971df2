/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/image/cuda_utils.hpp"

#include "engine/core/image/image.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/image/io.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/tensor/utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

// TODO(dweikersdorf) Use image convert and iterate function

TEST(CudaImage, Resize) {
  Image3ub image;
  ASSERT_TRUE(LoadPng("engine/gems/image/data/stairs.png", image));
  CudaImage3ub gpu_image(image.rows(), image.cols());
  Copy(image, gpu_image);

  CudaImage3ub gpu_result(image.rows() / 4, image.cols() / 4);

  nppiResize_8u_C3R(gpu_image.element_wise_begin(), gpu_image.getStride(), ImageNppSize(gpu_image),
                    ImageNppiRect(gpu_image), gpu_result.element_wise_begin(),
                    gpu_result.getStride(), ImageNppSize(gpu_result), ImageNppiRect(gpu_result),
                    NPPI_INTER_CUBIC);

  Image3ub result(gpu_result.rows(), gpu_result.cols());
  Copy(gpu_result, result);
  ASSERT_TRUE(SavePng(result, "/tmp/CudaImage_test.png"));
}

TEST(CudaImage, CopyHostDeviceHost) {
  Image3ub image_1(238, 723);
  for (int row = 0; row < image_1.rows(); row++) {
    for (int col = 0; col < image_1.cols(); col++) {
      image_1(row, col) = Pixel3ub{static_cast<byte>(row % 256), static_cast<byte>(col % 256),
                                   static_cast<byte>(row * (col) % 256)};
    }
  }

  CudaImage3ub image_2(image_1.rows(), image_1.cols());
  Copy(image_1, image_2);

  Image3ub image_3(image_2.rows(), image_2.cols());
  Copy(image_2, image_3);

  for (int row = 0; row < image_1.rows(); row++) {
    for (int col = 0; col < image_1.cols(); col++) {
      const auto expected = image_1(row, col);
      const auto actual = image_3(row, col);
      ASSERT_EQ(expected[0], actual[0]);
      ASSERT_EQ(expected[1], actual[1]);
      ASSERT_EQ(expected[2], actual[2]);
    }
  }
}

TEST(tensor_encoder, DenormalizeImage) {
  CudaImage3f src{256, 512};
  CudaImage3f src2{256, 512};
  Image3ub cpu_img{256, 512};
  CudaImage3ub dst{256, 512};
  Tensor3f tensor(256, 512, 3);

  Fill(tensor, 1.0f);
  Copy(ImageConstView3f(
           CpuBufferConstView(tensor.data().pointer().get(), src.rows() * src.cols() * 12),
           src.rows(), src.cols()),
       src);
  DenormalizeImage(src.view(), ImageNormalizationMode::kPositiveNegativeUnit, src2, dst.view());
  Copy(dst, cpu_img);

  // TODO(dweikersdorf) Use image iterate function
  for (int i = 0; i < cpu_img.rows(); i++) {
    for (int j = 0; j < cpu_img.cols(); j++) {
      for (int k = 0; k < 3; ++k) {
        EXPECT_EQ(cpu_img(i, j)[k], 255);
      }
    }
  }

  Fill(tensor, 0.0f);
  Copy(ImageConstView3f(
           CpuBufferConstView(tensor.data().pointer().get(), src.rows() * src.cols() * 12),
           src.rows(), src.cols()),
       src);
  DenormalizeImage(src.view(), ImageNormalizationMode::kPositiveNegativeUnit, src2, dst.view());
  Copy(dst, cpu_img);
  // TODO(dweikersdorf) Use image iterate function
  for (int i = 0; i < cpu_img.rows(); i++) {
    for (int j = 0; j < cpu_img.cols(); j++) {
      for (int k = 0; k < 3; ++k) {
        EXPECT_EQ(cpu_img(i, j)[k], 128);
      }
    }
  }

  Fill(tensor, -1.0f);
  Copy(ImageConstView3f(
           CpuBufferConstView(tensor.data().pointer().get(), src.rows() * src.cols() * 12),
           src.rows(), src.cols()),
       src);
  DenormalizeImage(src.view(), ImageNormalizationMode::kPositiveNegativeUnit, src2, dst.view());
  Copy(dst, cpu_img);
  for (int i = 0; i < cpu_img.rows(); i++) {
    for (int j = 0; j < cpu_img.cols(); j++) {
      for (int k = 0; k < 3; ++k) {
        EXPECT_EQ(cpu_img(i, j)[k], 0);
      }
    }
  }
}

TEST(CudaImage, FillElementsWithZero) {
  Image1ub image(10, 20);
  for (int i = 0; i < image.rows(); i++) {
    for (int j = 0; j < image.cols(); j++) {
      image(i, j) = static_cast<uint8_t>(i * j);
    }
  }
  CudaImage1ub cuda_image(10, 20);
  Copy(image, cuda_image);
  FillElementsWithZero(cuda_image.view());
  Copy(cuda_image, image);
  for (int i = 0; i < image.rows(); i++) {
    for (int j = 0; j < image.cols(); j++) {
      EXPECT_EQ(image(i, j), 0);
    }
  }
}

TEST(CudaImage, ResizeImageToRoi) {
  CudaImage1ub img1(10, 20);
  CudaImage1ub img2(10, 20);
  Image1ub cpu_img(10, 20);
  RegionOfInterest dst_roi{{0, 0}, {5, 10}};

  FillElementsWithZero(img1.view());
  FillElementsWithZero(img2.view());
  Copy(img1, cpu_img);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 4; ++j) {
      cpu_img(i, j) = 100;
    }
  }
  Copy(cpu_img, img1);
  ResizeImageToRoi(img1.const_view(), dst_roi, NPPI_INTER_NN, img2.view());
  Copy(img2, cpu_img);
  EXPECT_EQ(cpu_img(0, 0), 100);
  for (int i = 0; i < cpu_img.rows(); ++i) {
    for (int j = 0; j < cpu_img.cols(); ++j) {
      const uint8_t exp_val = i < 1 && j < 2 ? 100 : 0;
      EXPECT_EQ(exp_val, cpu_img(i, j));
    }
  }
}

TEST(CudaImage, CropImageToRoi) {
  CudaImage1ub img1(10, 20);
  CudaImage1ub img2(10, 20);
  Image1ub cpu_img(10, 20);

  RegionOfInterest src_roi{{1, 0}, {5, 10}};
  RegionOfInterest dst_roi{{5, 10}, {5, 10}};

  FillElementsWithZero(img1.view());
  FillElementsWithZero(img2.view());
  Copy(img1, cpu_img);
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 4; ++j) {
      cpu_img(i, j) = 100;
    }
  }
  Copy(cpu_img, img1);
  CropImageToRoi(img1.const_view(), src_roi, dst_roi, NPPI_INTER_NN, img2.view());
  Copy(img2, cpu_img);
  for (int i = 0; i < cpu_img.rows(); ++i) {
    for (int j = 0; j < cpu_img.cols(); ++j) {
      const uint8_t exp_val = (5 <= i && i < 6 && 10 <= j && j < 14) ? 100 : 0;
      EXPECT_EQ(cpu_img(i, j), exp_val);
    }
  }
}

TEST(CudaImage, ResizeWithAspectRatio) {
  CudaImage1ub img1(8, 16);
  CudaImage1ub img2(8, 8);
  Image1ub cpu_img1(8, 16);
  Image1ub cpu_img2(8, 8);
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 16; ++j) {
      cpu_img1(i, j) = 100;
    }
  }
  Copy(cpu_img1, img1);
  ResizeWithAspectRatio(img1.const_view(), NPPI_INTER_LINEAR, img2.view());
  Copy(img2, cpu_img2);
  for (int i = 0; i < cpu_img2.rows(); ++i) {
    for (int j = 0; j < cpu_img2.cols(); ++j) {
      const uint8_t exp_val = i > 1 && i < 6 ? 100 : 0;
      EXPECT_EQ(exp_val, cpu_img2(i, j));
    }
  }
}

TEST(CudaImage, ConvImageFilter) {
  const int rows = 10;
  const int cols = 10;
  const int filter_rows = 3;
  const int filter_cols = 3;

  CudaImage1f img1(rows, cols);
  CudaImage1f img2(rows, cols);
  CudaImage1f filter(filter_rows, filter_cols);

  Image1f cpu_img(rows, cols);
  Image1f exp_img(rows, cols);
  Image1f cpu_filter(filter_rows, filter_cols);

  FillElementsWithZero(img1.view());
  Copy(img1, cpu_img);
  cpu_img(1, 1) = 1.0f;
  Copy(cpu_img, img1);

  FillElementsWithZero(img2.view());
  Copy(img2, exp_img);

  cpu_filter(0, 0) = 1.0f;
  cpu_filter(0, 1) = 2.0f;
  cpu_filter(0, 2) = 3.0f;
  cpu_filter(1, 0) = 4.0f;
  cpu_filter(1, 1) = 5.0f;
  cpu_filter(1, 2) = 6.0f;
  cpu_filter(2, 0) = 7.0f;
  cpu_filter(2, 1) = 8.0f;
  cpu_filter(2, 2) = 9.0f;

  Copy(cpu_filter, filter);

  Vector2i anchor{1, 1};
  ConvImageFilter(filter.const_view(), anchor, img1.const_view(), img2.view());

  Copy(img2, cpu_img);

  exp_img(0, 0) = 1.0f;
  exp_img(0, 1) = 2.0f;
  exp_img(0, 2) = 3.0f;
  exp_img(1, 0) = 4.0f;
  exp_img(1, 1) = 5.0f;
  exp_img(1, 2) = 6.0f;
  exp_img(2, 0) = 7.0f;
  exp_img(2, 1) = 8.0f;
  exp_img(2, 2) = 9.0f;

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      EXPECT_EQ(cpu_img(i, j), exp_img(i, j)) << i << " " << j;
    }
  }
}

TEST(tensor_encoder, NormalizeImage) {
  CudaImage3ub src{256, 512};
  CudaImage3f dst{256, 512};
  Image3ub cpu_img{256, 512};
  Tensor3f tensor(256, 512, 3);
  Fill(tensor, 0.0f);
  auto tensor_view = ImageView3f(
      CpuBufferView(tensor.data().pointer().get(), dst.rows() * dst.cols() * 3 * sizeof(float)),
      dst.rows(), dst.cols());

  FillElementsWithZero(src.view());
  Copy(src, cpu_img);
  const int pos_row = 10;
  const int pos_col = 15;
  cpu_img(pos_row, pos_col) = Pixel3ub{127, 128, 255};
  Copy(cpu_img, src);

  const float epsilon = 1e-6;
  NormalizeImage(src.const_view(), isaac::ImageNormalizationMode::kPositiveNegativeUnit,
                 dst.view());
  Copy(dst, tensor_view);
  EXPECT_NEAR(tensor(pos_row, pos_col, 0), -(1.0 / 255.0), epsilon);
  EXPECT_NEAR(tensor(pos_row, pos_col, 1), (1.0 / 255.0), epsilon);
  EXPECT_NEAR(tensor(pos_row, pos_col, 2), 1, epsilon);
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 512; j++) {
      if (i == pos_row && j == pos_col) {
        continue;
      }

      EXPECT_EQ(tensor(i, j, 0), -1);
      EXPECT_EQ(tensor(i, j, 1), -1);
      EXPECT_EQ(tensor(i, j, 2), -1);
    }
  }

  NormalizeImage(src.const_view(), isaac::ImageNormalizationMode::kZeroUnit, dst.view());
  Copy(dst, tensor_view);
  EXPECT_NEAR(tensor(pos_row, pos_col, 0), (127.0 / 255.0), epsilon);
  EXPECT_NEAR(tensor(pos_row, pos_col, 1), (128.0 / 255.0), epsilon);
  EXPECT_NEAR(tensor(pos_row, pos_col, 2), 1, epsilon);
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 512; j++) {
      if (i == pos_row && j == pos_col) {
        continue;
      }

      EXPECT_EQ(tensor(i, j, 0), 0);
      EXPECT_EQ(tensor(i, j, 1), 0);
      EXPECT_EQ(tensor(i, j, 2), 0);
    }
  }

  NormalizeImage(src.const_view(), isaac::ImageNormalizationMode::kCast, dst.view());
  Copy(dst, tensor_view);
  EXPECT_NEAR(tensor(pos_row, pos_col, 0), 127.0, epsilon);
  EXPECT_NEAR(tensor(pos_row, pos_col, 1), 128.0, epsilon);
  EXPECT_NEAR(tensor(pos_row, pos_col, 2), 255.0, epsilon);
  for (int i = 0; i < 256; i++) {
    for (int j = 0; j < 512; j++) {
      if (i == pos_row && j == pos_col) {
        continue;
      }

      EXPECT_EQ(tensor(i, j, 0), 0);
      EXPECT_EQ(tensor(i, j, 1), 0);
      EXPECT_EQ(tensor(i, j, 2), 0);
    }
  }
}  // namespace isaac

}  // namespace isaac
