/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/image/processing.hpp"

#include <random>
#include "engine/core/image/image.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Convolve2DSeparableKernel, differentDeltas) {
  std::vector<int> input_sizes{600, 400, 201, 50, 41};
  Vector<float, 5> k;
  k << 0.1f, 0.2f, 0.4f, 0.2f, 0.1f;
  std::vector<uint32_t> deltas{1, 2, 3, 4};
  for (int& rows : input_sizes) {
    for (int& cols : input_sizes) {
      Image3ub input(rows, cols);
      FillElements(input, uint8_t{100});
      for (uint32_t& row_delta : deltas) {
        for (uint32_t& col_delta : deltas) {
          Image3ub output_image;
          Convolve2DSeparableKernel(input, k, k, row_delta, col_delta, output_image);
          for (auto p = output_image.element_wise_begin(); p != output_image.element_wise_end();
               ++p) {
            EXPECT_EQ(*p, 100);
          }
        }
      }
    }
  }
}

TEST(Downsample, differentDeltas) {
  std::vector<int> input_sizes{600, 701};
  std::vector<int> output_sizes{400, 201, 50, 41};
  for (int& in_rows : input_sizes) {
    for (int& in_cols : input_sizes) {
      Image3ub input(in_rows, in_cols);
      FillElements(input, uint8_t{100});
      for (int& out_rows : output_sizes) {
        for (int& out_cols : output_sizes) {
          Image3ub output_image;
          Downsample(input, Vector2i{out_rows, out_cols}, output_image);
          for (auto p = output_image.element_wise_begin(); p != output_image.element_wise_end();
               ++p) {
            EXPECT_EQ(*p, 100);
          }
        }
      }
    }
  }
}

TEST(Correlation, image_size) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;

  image_1.resize(3, 3);
  image_2.resize(4, 4);
  // Mismatch image size should fail
  bool success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, false);

  image_1.resize(4, 4);
  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = count;
        image_2(i, j)[k] = count;
        ++count;
      }
    }
  }

  // Images should be correlated and of the same size.
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_EQ(correlation, 1.0);
}

TEST(Correlation, anti_correlation) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = count;
        image_2(i, j)[k] = -count;
        ++count;
      }
    }
  }
  // Images should be anticorrelated
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_EQ(correlation, -1.0);
}

TEST(Correlation, mean_shift) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = count + 100.0;
        image_2(i, j)[k] = count;
        ++count;
      }
    }
  }
  // Images should be correlated
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_EQ(correlation, 1.0);
}

TEST(Correlation, scale_shift) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = count * 100.0;
        image_2(i, j)[k] = count;
        ++count;
      }
    }
  }
  // Images should be correlated
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_EQ(correlation, 1.0);
}

TEST(Correlation, noise) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  std::random_device device;
  std::mt19937 generator(device());
  std::uniform_int_distribution<> distribution(1, 10);

  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = count;
        image_2(i, j)[k] = count;
        ++count;
      }
    }
  }

  // Test different noise levels
  for (int num_test = 1; num_test <= 10; ++num_test) {
    double count = 0.0;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        for (int k = 0; k < 4; ++k) {
          image_1(i, j)[k] = count + num_test * distribution(generator);
          ++count;
        }
      }
    }
    // Images should be correlated to some degree but not perfectly
    success = ImageCorrelation(image_1, image_2, correlation);
    EXPECT_EQ(success, true);
    EXPECT_LT(correlation, 1.0);
    EXPECT_GT(correlation, 0.0);
  }
}

TEST(Correlation, random) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  std::random_device device;
  std::mt19937 generator(device());
  std::uniform_int_distribution<> distribution(-200, 200);
  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = distribution(generator);
        image_2(i, j)[k] = distribution(generator);
        ++count;
      }
    }
  }
  // Random data should have some non perfect correlation
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_LT(correlation, 1.0);
  EXPECT_GT(correlation, -1.0);
}

TEST(Correlation, channel_swap) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  std::random_device device;
  std::mt19937 generator(device());
  std::uniform_int_distribution<> distribution(-200, 200);
  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = distribution(generator);
        image_2(i, j)[3 - k] = image_1(i, j)[k];
        ++count;
      }
    }
  }
  // Images correlation should be effectively a noisy signal
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_LT(correlation, 1.0);
  EXPECT_GT(correlation, -1.0);
}

TEST(Correlation, uncorrelated) {
  Image4d image_1;
  Image4d image_2;
  double correlation = 0.0;
  bool success = false;

  image_1.resize(4, 4);
  image_2.resize(4, 4);

  std::random_device device;
  std::mt19937 generator(device());
  std::poisson_distribution<> distribution_1(10);
  std::normal_distribution<> distribution_2(2000, 5);
  double count = 0.0;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      for (int k = 0; k < 4; ++k) {
        image_1(i, j)[k] = distribution_1(generator);
        image_2(i, j)[3 - k] = distribution_2(generator);
        ++count;
      }
    }
  }
  // In a perfect world this would be a zero correlation, but given finite
  // sampling and other realities that is not possible to acheive.
  success = ImageCorrelation(image_1, image_2, correlation);
  EXPECT_EQ(success, true);
  EXPECT_LT(correlation, 1.0);
  EXPECT_GT(correlation, -1.0);
}
}  // namespace isaac
