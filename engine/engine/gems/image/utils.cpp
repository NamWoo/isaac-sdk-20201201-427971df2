/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "utils.hpp"

#include <algorithm>
#include <utility>

#include "engine/core/math/utils.hpp"

namespace isaac {

template <typename K, typename Container,
          typename std::enable_if<std::is_floating_point<K>::value, int>::type>
void ConvertDisparityToDepth(const ImageBase<K, 1, Container>& disparity_img, K baseline,
                             K focal_length_px, K min_depth, K max_depth, Image<K, 1>& depth_img) {
  ASSERT(min_depth > K(0), "min depth must be greater than 0");
  ASSERT(max_depth > min_depth, "max depth must be greater than min depth");
  depth_img.resize(disparity_img.dimensions());
  const K c = baseline * focal_length_px;
  ASSERT(c > K(0), "c must be greater than 0");
  const K min_disparity = c / max_depth;
  const K max_disparity = c / min_depth;
  Convert(disparity_img, depth_img, [=](K disp) {
    return c / std::min(std::max(disp, min_disparity), max_disparity);
  });
}

void JoinTwoImagesSideBySide(const ImageConstView3ub& left, const ImageConstView3ub& right,
                             Image3ub& joint) {
  constexpr int kChannels = 3;

  const size_t left_rows = left.rows();
  const size_t left_cols = left.cols();
  const size_t right_rows = right.rows();
  const size_t right_cols = right.cols();

  ASSERT(left_rows == right_rows, "image rows mismatch, left: %zu, right: %zu", left_rows,
         right_rows);
  const size_t joint_rows = left_rows;
  const size_t joint_cols = left_cols + right_cols;
  joint.resize(joint_rows, joint_cols);

  for (size_t i = 0; i < joint_rows; i++) {
    unsigned char* joint_row_begin = joint.row_pointer(i);
    std::memcpy(joint_row_begin, left.row_pointer(i), left_cols * kChannels);
    std::memcpy(joint_row_begin + left_cols * kChannels,
                right.row_pointer(i), right_cols * kChannels);
  }
}

void SplitImages(const ImageConstView3ub& joint, Image3ub& left, Image3ub& right) {
  constexpr int kChannels = 3;

  const size_t rows = joint.rows();
  const size_t cols = joint.cols();

  ASSERT(cols % 2 == 0, "the width of the joint images is not a multiple of 2");
  const size_t split_cols = cols / 2;
  const size_t elements_per_row = split_cols * kChannels;

  left.resize(rows, split_cols);
  right.resize(rows, split_cols);

  for (size_t i = 0; i < rows; i++) {
    const unsigned char* joint_row_begin = joint.row_pointer(i);
    std::memcpy(left.row_pointer(i), joint_row_begin, elements_per_row);
    std::memcpy(right.row_pointer(i), joint_row_begin + elements_per_row, elements_per_row);
  }
}

// -------------------------------------------------------------------------------------------------

template void ConvertDisparityToDepth(const ImageConstView1f& disparity_img, float baseline,
                                      float focal_length_px, float min_depth, float max_depth,
                                      Image1f& depth_img);

}  // namespace isaac
