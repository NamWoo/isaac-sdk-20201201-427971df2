/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "engine/core/buffers/algorithm.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/image/utils.hpp"

namespace isaac {

// Computes quickly (linear time) a distance map with up to 12% errors.
// It computes the minimum distance using horizontal, vertical and diagonal line with the
// approximation that sqrt(2) = 1.5
// Take an Image1ub as input as well as the value of the sources.
// The distance is computed in pixel and will be scaled by the pixel size.
// If smoothing is greater than zero, then a blur will be applied, it will produce a smoother
// distance map but the distance near the extremum will be inacurate.
// The distance will be capped at max_distance.
template <typename K>
Image<K, 1> QuickDistanceMapApproximated(const ImageConstView1ub& img, uint8_t sources,
                                         K pixel_size,
                                         K max_distance = std::numeric_limits<K>::max());

// Computes quickly (linear time) a distance map with up to 2% errors.
// Take an Image1ub as input as well as the value of the sources.
// The distance is computed in pixel and will be scaled by the pixel size.
// The higher Res, the slower but the more accurate the map will be. res must be greater or equal
// to 2.
// The distance will be capped at max_distance.
template <int Res = 2, typename K>
Image<K, 1> QuickDistanceMap(const ImageConstView1ub& img, uint8_t sources, K pixel_size,
                             K max_distance = std::numeric_limits<K>::max());

// Computes a distance map.
// Take an Image1ub as input as well as the value of the sources.
// The distance is computed in pixel and will be scaled by the pixel size.
// The distance will be capped at max_distance.
template <typename K>
Image<K, 1> DistanceMap(const ImageConstView1ub& img, uint8_t sources, K pixel_size,
                        K max_distance = std::numeric_limits<K>::max());

// -------------------------------------------------------------------------------------------------

template <typename K>
Image<K, 1> QuickDistanceMapApproximated(const ImageConstView1ub& img, uint8_t sources,
                                         K pixel_size, K max_distance) {
  const int rows = img.rows();
  const int cols = img.cols();
  Image<K, 1> map(rows, cols);
  std::vector<bool> seen(img.num_pixels(), false);
  std::vector<int> lists[4];
  // Initialize the seed for each pixel matching sources.
  for (int pixel = 0; pixel < img.num_pixels(); pixel++) {
    if (img[pixel] == sources) {
      lists[0].push_back(pixel);
    }
  }
  size_t num_opens = lists[0].size();
  if (num_opens == 0) {
    FillPixels(map, max_distance);
    return map;
  }
  for (int distance = 0; num_opens > 0; distance++) {
    for (int pixel : lists[distance % 4]) {
      if (seen[pixel]) continue;
      seen[pixel] = true;
      map[pixel] = std::min(max_distance, distance * pixel_size / K(2));
      const int row = pixel / cols;
      const int col = pixel % cols;
      constexpr int kDir[9] = {1, 0, -1, 0, 1, -1, -1, 1, 1};
      constexpr int kDist[8] = {2, 2, 2, 2, 3, 3, 3, 3};
      for (int dir = 0; dir < 8; dir++) {
        const int nrow = row + kDir[dir];
        const int ncol = col + kDir[dir + 1];
        const int npixel = nrow * cols + ncol;
        if (nrow < 0 || nrow >= rows || ncol < 0 || ncol >= cols || seen[npixel]) {
          continue;
        }
        lists[(distance + kDist[dir]) % 4].push_back(npixel);
        num_opens++;
      }
    }
    num_opens -= lists[distance % 4].size();
    lists[distance % 4].clear();
  }
  return map;
}


template <int Res, typename K>
Image<K, 1> QuickDistanceMap(const ImageConstView1ub& img, uint8_t sources, K pixel_size,
                             K max_distance) {
  static_assert(Res >= 2, "res must be greater than 2");
  constexpr int kSize = 2 + Res;
  const int rows = img.rows();
  const int cols = img.cols();
  const int size = rows * cols;
  // Store the final distance map
  Image<K, 1> map(rows, cols);
  // Store the index of the closest obstacle detected so far
  std::vector<int> obstacles(size);
  // Maintain a rolling a list of pixel to process at a given distance
  std::vector<int> lists[kSize];
  // Pre allocate enough memory for most of the distances.
  for (int i = 1; i < kSize; i++) {
    lists[i].reserve(rows + cols);
  }
  constexpr int kNotOpen = -2;
  constexpr int kProcessed = -1;
  // Initialize the seed for each pixel matching sources and prefill the map
  for (int pixel = 0; pixel < size; pixel++) {
    if (img[pixel] == sources) {
      lists[0].push_back(pixel);
      obstacles[pixel] = pixel;
      map[pixel] = K(0);
    } else {
      map[pixel] = max_distance;
      obstacles[pixel] = kNotOpen;
    }
  }
  size_t num_opens = lists[0].size();
  if (num_opens == 0) {
    return map;
  }
  for (int distance = 0; num_opens > 0; distance++) {
    const int mod_distance = distance % kSize;
    for (size_t idx = 0; idx < lists[mod_distance].size(); idx++) {
      const int pixel = lists[mod_distance][idx];
      const int obstacle = obstacles[pixel];
      if (obstacle == kProcessed) continue;
      obstacles[pixel] = kProcessed;
      const int row = pixel / cols;
      const int col = pixel % cols;
      const int ocol = obstacle % cols;
      const int orow = obstacle / cols;
      // Function that checks if a pixel need to be added to the queue and adds it if needed.
      auto expand = [&] (int pixel, int row, int col) {
        if (obstacles[pixel] == kProcessed) return;
        const int dcol = col - ocol;
        const int drow = row - orow;
        const K cur_dist_pixel = std::sqrt(static_cast<K>(dcol * dcol + drow * drow));
        const K cur_dist = cur_dist_pixel * pixel_size;
        // If the current distance is more than the existing one there is no point expanding this
        // node.
        if (cur_dist > map[pixel]) return;
        map[pixel] = cur_dist;
        const int dist = static_cast<int>(K(Res) * cur_dist_pixel + K(0.5));
        lists[dist % kSize].push_back(pixel);
        obstacles[pixel] = obstacle;
        num_opens++;
      };
      // Unwrap the loop to only do the check needed.
      if (row > 0)        expand(pixel - cols, row - 1, col);
      if (row + 1 < rows) expand(pixel + cols, row + 1, col);
      if (col > 0)        expand(pixel - 1,    row,     col - 1);
      if (col + 1 < cols) expand(pixel + 1,    row,     col + 1);
    }
    num_opens -= lists[mod_distance].size();
    lists[mod_distance].clear();
  }
  return map;
}

template <typename K>
Image<K, 1> DistanceMap(const ImageConstView1ub& img, uint8_t sources,
                        K pixel_size, K max_distance) {
  // Create the output image
  Image<K, 1> dist(img.rows(), img.cols());
  // For the first round we will compute the square cell distance. So we need to compute the
  // corresponding maximum based on cell size and max distance.
  const K max_cells = max_distance / pixel_size;
  FillPixels(dist, max_cells * max_cells);
  const int rows = img.rows();
  const int cols = img.cols();
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      if (img(row, col) != sources) {
        continue;
      }
      for (int nrow = 0; nrow < rows; nrow++) {
        const int drow = row - nrow;
        const int drow_sq = drow * drow;
        for (int ncol = 0; ncol < cols; ncol++) {
          // Store minimal square cell distance
          const int dcol = col - ncol;
          const K distance = static_cast<K>(dcol * dcol + drow_sq);
          K& dist_value = dist(nrow, ncol);
          dist_value = std::min(dist_value, distance);
        }
      }
    }
  }
  // Convert square cell distance to desired units
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      dist(row, col) = pixel_size * std::sqrt(dist(row, col));
    }
  }
  return dist;
}

}  // namespace isaac
