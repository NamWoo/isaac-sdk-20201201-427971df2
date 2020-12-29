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
#include <array>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/interpolation/linear.hpp"
#include "engine/gems/interpolation/utils.hpp"

namespace isaac {

// Helper class to approximate a bivariate function with a 2d map of the function values
//  K: type of the co-domain (function output)
//  F: type of a function/object that accept the syntax: obj(x, y) -> K
template <typename K, bool Precompute = true, typename F = std::function<K(int, int)>>
class BilinearApproximatedFunction {
 public:
  // Constructor
  //  f: The function to approximate (need to be callable with any (row, col) in [0,rows[x[0,cols[)
  //  rows: the number of rows of the domain
  //  cols: the number of columns of the domain
  BilinearApproximatedFunction(F f, int rows, int cols);

  // Gets the approximate function value at the given value. This will use bilinear interpolation
  // inside cells, and linear continuation outside of the domain.
  K operator()(K x, K y);

 private:
  // Return the coefficient matrix. If not already computed it computes it first.
  const Matrix2<K>& coefficients(int row, int col);

  // Computes the coefficients for a given cell;
  void computeCoefficients(int row, int col);

  const F f_;
  std::vector<bool> cached_;
  Tensor<Matrix2<K>, 2> coefficients_;
};

// -------------------------------------------------------------------------------------------------

// Implementation of the BilinearApproximatedFunction
template <typename K, bool Precompute, typename F>
BilinearApproximatedFunction<K, Precompute, F>::BilinearApproximatedFunction(F f, int rows,
                                                                             int cols)
    : f_(std::move(f)) {
  ASSERT(rows > 1 && cols > 1, "Invalid boundary (Size of the cache: %zdx%zd", rows, cols);
  coefficients_.resize(rows - 1, cols - 1);
  if (Precompute) {
    for (int row = 0; row < coefficients_.dimensions()[0]; row++) {
      for (int col = 0; col < coefficients_.dimensions()[1]; col++) {
        computeCoefficients(row, col);
      }
    }
  } else {
    cached_.resize(coefficients_.element_count(), false);
  }
}

template <typename K, bool Precompute, typename F>
K BilinearApproximatedFunction<K, Precompute, F>::operator()(K x, K y) {
  const int row = x < K(0) ? 0 : std::min(static_cast<int>(x), coefficients_.dimensions()[0] - 1);
  const int col = y < K(0) ? 0 : std::min(static_cast<int>(y), coefficients_.dimensions()[1] - 1);
  const K dx = x - static_cast<K>(row);
  const K dy = y - static_cast<K>(col);

  const Matrix2<K>& coeffs = coefficients(row, col);
  return Vector2<K>(K(1), dx).transpose() * coeffs * Vector2<K>(K(1), dy);
}

template <typename K, bool Precompute, typename F>
const Matrix2<K>& BilinearApproximatedFunction<K, Precompute, F>::coefficients(int row,
                                                                               int col) {
  const int index = row * coefficients_.dimensions()[1] + col;
  if (!Precompute && !cached_[index]) {
    cached_[index] = true;
    computeCoefficients(row, col);
  }
  return coefficients_(row, col);
}

template <typename K, bool Precompute, typename F>
void BilinearApproximatedFunction<K, Precompute, F>::computeCoefficients(int row, int col) {
  const K f00 = f_(row, col);
  const K f10 = f_(row + 1, col);
  const K f01 = f_(row, col + 1);
  const K f11 = f_(row + 1, col + 1);
  Matrix2<K>& mat = coefficients_(row, col);
  mat(0, 0) = f00;
  mat(1, 0) = f10 - f00;
  mat(0, 1) = f01 - f00;
  mat(1, 1) = f11 + f00 - f01 - f10;
}

}  // namespace isaac
