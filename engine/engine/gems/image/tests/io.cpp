/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/image/image.hpp"

#include "engine/gems/image/io.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(images, save_load_1ub) {
  Image1ub expected(162, 93);
  for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      expected(i,j) = static_cast<unsigned char>(i * j);
    }
  }
  ASSERT_TRUE(SavePng(expected, "/tmp/test1.png"));

  Image1ub actual;
  ASSERT_TRUE(LoadPng("/tmp/test1.png", actual));
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      ASSERT_EQ(expected(i,j), actual(i,j));
    }
  }
}

TEST(images, save_load_3ub) {
  Image3ub expected(162, 93);
  for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      expected(i,j) = Pixel3ub{
        static_cast<unsigned char>(i),
        static_cast<unsigned char>(j),
        static_cast<unsigned char>(i * j),
      };
    }
  }
  ASSERT_TRUE(SavePng(expected, "/tmp/test3.png"));

  Image3ub actual;
  ASSERT_TRUE(LoadPng("/tmp/test3.png", actual));
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      ASSERT_EQ(expected(i,j)[0], actual(i,j)[0]);
      ASSERT_EQ(expected(i,j)[1], actual(i,j)[1]);
      ASSERT_EQ(expected(i,j)[2], actual(i,j)[2]);
    }
  }
}

TEST(images, save_load_jpg_1ub) {
  Image1ub expected(162, 93);
  for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      expected(i,j) = static_cast<unsigned char>(i * j);
    }
  }
  ASSERT_TRUE(SaveJpeg(expected, "/tmp/test1.jpg"));

  Image1ub actual;
  ASSERT_TRUE(LoadJpeg("/tmp/test1.jpg", actual));
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  int matches = 0;
    for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      if(expected(i,j) == actual(i,j)) matches++;
    }
  }
  ASSERT_EQ(matches, 1171);
}

TEST(images, save_load_jpg_3ub) {
  Image3ub expected(162, 93);
  for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      expected(i,j) = Pixel3ub{
        static_cast<unsigned char>(i),
        static_cast<unsigned char>(j),
        static_cast<unsigned char>(i * j),
      };
    }
  }
  ASSERT_TRUE(SaveJpeg(expected, "/tmp/test3.jpg"));

  Image3ub actual;
  ASSERT_TRUE(LoadJpeg("/tmp/test3.jpg", actual));
  ASSERT_EQ(actual.rows(), expected.rows());
  ASSERT_EQ(actual.cols(), expected.cols());
  int matches = 0;
    for (int i=0; i<expected.rows(); i++) {
    for (int j=0; j<expected.cols(); j++) {
      if(expected(i,j)[0] == actual(i,j)[0]) matches++;
      if(expected(i,j)[1] == actual(i,j)[1]) matches++;
      if(expected(i,j)[2] == actual(i,j)[2]) matches++;
    }
  }
  ASSERT_EQ(matches, 1825);
}

TEST(images, load_3ub_to_1ub) {
  Image1ub img;
  ASSERT_TRUE(LoadPng("engine/gems/image/data/room.png", img));
  EXPECT_EQ(img.rows(), 117);
  EXPECT_EQ(img.cols(), 83);
}

TEST(images, load_3ub_to_3ub) {
  Image3ub img;
  ASSERT_TRUE(LoadPng("engine/gems/image/data/room.png", img));
  EXPECT_EQ(img.rows(), 117);
  EXPECT_EQ(img.cols(), 83);
}

TEST(Io, FailLoad) {
  Image3ub img;
  EXPECT_FALSE(LoadPng("definitely/not/an/image.gif", img));
}

TEST(Io, FailSave) {
  Image3ub img(10, 20);
  EXPECT_FALSE(SavePng(img, "definitely/not/an/image.gif"));
}

}  // namespace isaac
