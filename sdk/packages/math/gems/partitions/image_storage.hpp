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

#include "engine/core/assert.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {
namespace partitions {

// A storage which uses a single rectangular dense array (in form of an image) to store grid data.
template <typename Cell, typename Index = int>
class ImageStorage {
 public:
  // Creates a new image storage with an empty image
  ImageStorage() : ImageStorage(0, 0) {}
  // Creates a new image storage with the given dimensions.
  ImageStorage(Index rows, Index cols) : image_(rows, cols) {}

  // The number of rows and columns in the map
  Index rows() const { return image_.rows(); }
  Index cols() const { return image_.cols(); }

  // Resizes the underlying image to the given dimensions.
  void resize(Index rows, Index cols) {
    image_.resize(rows, cols);
  }
  void resize(const Vector2<Index>& dimensions) {
    image_.resize(dimensions);
  }

  // Checks if a cell coordinate indicates a valid cell in the map
  bool isInRange(Index row, Index col) const {
    return 0 <= row && row < rows() && 0 <= col && col < cols();
  }
  bool isInRange(const Vector2i& cell) const {
    return isInRange(cell[0], cell[1]);
  }

  // Accesses the value stored at the cell coordinate (row, col)
  const Cell& at(Index row, Index col) const {
    return image_(row, col);
  }
  Cell& at(Index row, Index col) {
    return image_(row, col);
  }
  // Accesses the value stored for the given cell coordinate
  const Cell& at(const Vector2i& cell) const {
    return image_(cell[0], cell[1]);
  }
  Cell& at(const Vector2i& cell) {
    return image_(cell[0], cell[1]);
  }

  // Accesses the value stored at the cell coordinate (row, col)
  const Cell& operator()(Index row, Index col) const {
    return image_(row, col);
  }
  Cell& operator()(Index row, Index col) {
    return image_(row, col);
  }
  // Accesses the value stored for the given cell coordinate
  const Cell& operator()(const Vector2i& cell) const {
    return image_(cell[0], cell[1]);
  }
  Cell& operator()(const Vector2i& cell) {
    return image_(cell[0], cell[1]);
  }

  // Evaluates f(x) for every pixel `x` in this image.
  template <typename F>
  void iterate(F f) {
    for (int i = 0; i < image_.num_pixels(); i++) {
      f(image_[i]);
    }
  }
  template <typename F>
  void iterate(F f) const {
    for (int i = 0; i < image_.num_pixels(); i++) {
      f(image_[i]);
    }
  }

  // Evaluates f(i, x_i) for every pixel x_i in this image. Here `i` is the index of the pixel
  // based on a row-major storage order.
  template <typename F>
  void iterateIndexed(F f) {
    for (int i = 0; i < image_.num_pixels(); i++) {
      f(i, image_[i]);
    }
  }
  template <typename F>
  void iterateIndexed(F f) const {
    for (int i = 0; i < image_.num_pixels(); i++) {
      f(i, image_[i]);
    }
  }

  // Evaluates f(x, y) for every pixel `x` in this image and corresponding pixel `y` in the
  // other image. The dimensions of the other image must match.
  template <typename OtherCell, typename OtherIndex, typename F>
  void transform(ImageStorage<OtherCell, OtherIndex>& other, F f) const {
    ASSERT(other.rows() == rows(), "Number of rows does not match");
    ASSERT(other.cols() == cols(), "Number of cols does not match");
    auto other_view = other.image().view();
    for (int i = 0; i < image_.num_pixels(); i++) {
      f(image_[i], other_view[i]);
    }
  }

  // Gets access to the underlying image
  Image<Cell, 1>& image() { return image_; }
  const Image<Cell, 1>& image() const { return image_; }

 private:
  Image<Cell, 1> image_;
};

}  // namespace partitions
}  // namespace isaac
