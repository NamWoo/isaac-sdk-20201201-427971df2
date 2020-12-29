/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/image/image.hpp"

#include <vector>

#include "engine/core/math/types.hpp"
#include "engine/gems/math/float16.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Image, EmptyImage) {
  Image3ub img;
  ASSERT_TRUE(img.empty());
  EXPECT_EQ(img.rows(), 0);
  EXPECT_EQ(img.cols(), 0);
}

TEST(images, allocation) {
  Image3d t3;
  Image1d t1;
  t1 = Image1d(20, 40);
  t3 = Image3d(30, 60);
}

TEST(images, size) {
  Image3d t3(20, 40);
  Image1d t1(30, 60);
  EXPECT_EQ(t3.num_pixels(), 20 * 40);
  EXPECT_EQ(t3.rows(), 20);
  EXPECT_EQ(t3.cols(), 40);

  EXPECT_EQ(t1.num_pixels(), 30 * 60);
  EXPECT_EQ(t1.rows(), 30);
  EXPECT_EQ(t1.cols(), 60);
}

TEST(images, access) {
  Image3d t3(20, 40);
  Image1d t1(30, 60);
  t3(3, 5)[2] = 12.0;
  EXPECT_EQ(t3[40 * 3 + 5][2], 12.0);
  EXPECT_EQ(t3.element_wise_begin()[(40 * 3 + 5) * 3 + 2], 12.0);

  t1(3, 5) = 12.0;
  EXPECT_EQ(t1[60 * 3 + 5], 12.0);
}

TEST(images, const_access) {
  Image3d t3(20, 40);
  Image1d t1(30, 60);
  const Image1d& img1 = t1;
  const Image3d& img3 = t3;
  t3(3, 5)[2] = 12.0;
  EXPECT_EQ(img3(3, 5)[2], 12.0);
  EXPECT_EQ(img3(3, 5)(2), 12.0);
  EXPECT_EQ(img3[40 * 3 + 5][2], 12.0);
  EXPECT_EQ(img3[40 * 3 + 5](2), 12.0);
  EXPECT_EQ(img3.element_wise_begin()[(40 * 3 + 5) * 3 + 2], 12.0);

  t1(3, 5) = 12.0;
  EXPECT_EQ(img1(3, 5), 12.0);
  EXPECT_EQ(img1[60 * 3 + 5], 12.0);
}

TEST(images_view, size) {
  Image3d image3(20, 40);
  Image1d image1(30, 60);

  ImageView3d t3 = image3.view();
  ImageView1d t1 = image1.view();

  EXPECT_EQ(t3.num_pixels(), 20 * 40);
  EXPECT_EQ(t3.rows(), 20);
  EXPECT_EQ(t3.cols(), 40);

  EXPECT_EQ(t1.num_pixels(), 30 * 60);
  EXPECT_EQ(t1.rows(), 30);
  EXPECT_EQ(t1.cols(), 60);
}

TEST(const_images_view, size) {
  Image3d image3(20, 40);
  Image1d image1(30, 60);

  ImageConstView3d t3 = image3.const_view();
  ImageConstView1d t1 = image1.const_view();

  EXPECT_EQ(t3.num_pixels(), 20 * 40);
  EXPECT_EQ(t3.rows(), 20);
  EXPECT_EQ(t3.cols(), 40);

  EXPECT_EQ(t1.num_pixels(), 30 * 60);
  EXPECT_EQ(t1.rows(), 30);
  EXPECT_EQ(t1.cols(), 60);
}

TEST(images_view, access) {
  Image3d image3(20, 40);
  Image1d image1(30, 60);

  ImageView3d view3 = image3.view();
  ImageView1d view1 = image1.view();

  view3(3, 5)[2] = 12.0;
  EXPECT_EQ(view3[40 * 3 + 5][2], 12.0);
  EXPECT_EQ(view3.element_wise_begin()[(40 * 3 + 5) * 3 + 2], 12.0);

  view1(3, 5) = 12.0;
  EXPECT_EQ(view1[60 * 3 + 5], 12.0);
}

TEST(const_images_view, access) {
  Image3d image3(20, 40);
  Image1d image1(30, 60);

  ImageConstView3d view3 = image3.const_view();
  ImageConstView1d view1 = image1.const_view();

  image3(3, 5)[2] = 12.0;
  EXPECT_EQ(view3(3, 5)[2], 12.0);
  EXPECT_EQ(view3(3, 5)(2), 12.0);
  EXPECT_EQ(view3[40 * 3 + 5][2], 12.0);
  EXPECT_EQ(view3[40 * 3 + 5](2), 12.0);
  EXPECT_EQ(view3.element_wise_begin()[(40 * 3 + 5) * 3 + 2], 12.0);

  image1(3, 5) = 12.0;
  EXPECT_EQ(view1(3, 5), 12.0);
  EXPECT_EQ(view1[60 * 3 + 5], 12.0);
}

TEST(Image, ConvertViewToConstView) {
  Image3ub img(13, 15);
  ImageView3ub view1 = img.view();
  ImageConstView3ub view2 = view1;
  ASSERT_FALSE(view2.empty());
  EXPECT_EQ(view2.rows(), 13);
  EXPECT_EQ(view2.cols(), 15);
}

TEST(Image, EmptyView) {
  ImageView3ub view;
  ASSERT_TRUE(view.empty());
  EXPECT_EQ(view.rows(), 0);
  EXPECT_EQ(view.cols(), 0);
}

TEST(Image, EmptyConstView) {
  ImageConstView3ub view;
  ASSERT_TRUE(view.empty());
  EXPECT_EQ(view.rows(), 0);
  EXPECT_EQ(view.cols(), 0);
}

TEST(Image, CopyView) {
  Image3ub img(13, 15);
  ImageView3ub view1 = img.view();
  ImageView3ub view2;
  view2 = view1;
  ASSERT_FALSE(view2.empty());
  EXPECT_EQ(view2.rows(), 13);
  EXPECT_EQ(view2.cols(), 15);
}

TEST(Image, CopyConstView) {
  Image3ub img(13, 15);
  ImageConstView3ub view1 = img.const_view();
  ImageConstView3ub view2;
  view2 = view1;
  ASSERT_FALSE(view2.empty());
  EXPECT_EQ(view2.rows(), 13);
  EXPECT_EQ(view2.cols(), 15);
}

TEST(Image, Image1f16) {
  Image<float16, 1> image(20, 40);
  for (int i = 0; i < image.rows(); i++) {
    for (int j = 0; j < image.cols(); j++) {
      image(i, j) = static_cast<float16>(i * j);
    }
  }
  for (int i = 0; i < image.rows(); i++) {
    for (int j = 0; j < image.cols(); j++) {
      image(i, j) += 3.1;
    }
  }
  for (int i = 0; i < image.rows(); i++) {
    for (int j = 0; j < image.cols(); j++) {
      EXPECT_NEAR(image(i, j), 3.1 + static_cast<double>(i * j), 0.15);
    }
  }
}

TEST(Image, ResizeReallocatesIfNewDimsAreDifferent) {
  Image3ub image(10, 15);
  EXPECT_EQ(image.data().size(), 10*15*3);
  const uint8_t* image_ptr = image.element_wise_begin();
  image.resize(10, 15);
  EXPECT_EQ(image_ptr, image.element_wise_begin());
  image.resize(11, 20);
  EXPECT_EQ(image.data().size(), 11*20*3);
  image.resize(10, 15);
  EXPECT_EQ(image.data().size(), 10*15*3);
}

namespace {

void FooReadable(const ImageConstView1ub& view) {}

void FooWriteable(const ImageView1ub& view) {}

template <int N, typename Container>
void FooTemplated(const ImageBase<uint8_t, N, Container>& view) { }

}  // namespace

TEST(Image, CallFunctionReadable) {
  Image1ub img(10, 20);
  FooReadable(img);
  FooReadable(img.view());
  FooReadable(img.const_view());
}

TEST(Image, CallFunctionWriteable) {
  Image1ub img(10, 20);
  FooWriteable(img);
  FooWriteable(img.view());
  // FooWriteable(img.const_view());  // should not compile
}

TEST(Image, CallFunctionTemplated) {
  Image1ub img1(10, 20);
  FooTemplated(img1);
  FooTemplated(img1.view());
  FooTemplated(img1.const_view());
  Image3ub img3(10, 20);
  FooTemplated(img3);
  FooTemplated(img3.view());
  FooTemplated(img3.const_view());
}

TEST(Image, VectorOfImages) {
  std::vector<Image1ub> imgs;
  imgs.emplace_back(Image1ub(10, 20));
  EXPECT_EQ(imgs.size(), 1);
  imgs.reserve(10);
  EXPECT_EQ(imgs.capacity(), 10);
}

TEST(Image, RowPointer) {
  Image3ub img1(193, 211);
  unsigned char* begin1 = img1.element_wise_begin();
  for (int i = 0; i < img1.rows(); i++) {
    ASSERT_EQ(img1.row_pointer(i), begin1 + i * 633);
  }
  Image1f img2(193, 211);
  float* begin2 = img2.element_wise_begin();
  for (int i = 0; i < img2.rows(); i++) {
    ASSERT_EQ(img2.row_pointer(i), begin2 + i * 211);
  }
}

TEST(Image, TrivialStride) {
  Image3f img(193, 211);
  ASSERT_EQ(img.data().size(), 488676);
  ASSERT_EQ(img.getStride(), 2532);
}

TEST(Image, Pixels) {
  Image3ub image(32, 48);
  image(0, 0) = Pixel3ub{12, 117, 94};
  Pixel3ub pixel = image(0, 0);
  image(0, 1) = pixel;
}

TEST(CudaImage, Stride) {
  // FIXME The following expected stride values might be platform specific.
  CudaImage3ub gpu_image(10, 119);
  EXPECT_EQ(gpu_image.getStride(), 3*119);
  gpu_image.resize(480, 640);
  EXPECT_EQ(gpu_image.getStride(), 3*640);
  gpu_image.resize(10, 513);
  EXPECT_EQ(gpu_image.getStride(), 3*513);
  gpu_image.resize(10, 2049);
  EXPECT_EQ(gpu_image.getStride(), 3*2049);
  gpu_image.resize(540, 960);
  EXPECT_EQ(gpu_image.getStride(), 3*960);
}

TEST(CudaImage, NonTrivialStride) {
  CudaImage3f img(193, 211);
  ASSERT_EQ(img.data().size(), 193*211*3*4);
  ASSERT_EQ(img.getStride(), 2532);
}

}  // namespace isaac
