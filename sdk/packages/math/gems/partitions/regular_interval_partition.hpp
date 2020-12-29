/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/assert.hpp"
#include "engine/core/math/utils.hpp"
#include "packages/math/gems/partitions/regular_partition.hpp"

namespace isaac {
namespace partitions {

// A regular partition of an interval
template <typename Scalar, typename Index = int>
class RegularIntervalPartition {
 public:
  // Creates a regular partition covering of interval [min | max] with the given number of samples.
  // The first sample will be at coordinate `min` and the last sample at coordinate `max`.
  static RegularIntervalPartition FromSampleCount(Scalar min, Scalar max, Index num_samples) {
    ASSERT(num_samples >= 2, "Number of samples must be at least 2");
    return FromCellCount(min, max, num_samples - 1);
  }

  // Creates a regular partition covering of interval [min | max] with the given number of cells.
  // For example the interval [0 | 3] with 3 cells would result in a partition with points at
  // {0, 1, 2}
  static RegularIntervalPartition FromCellCount(Scalar min, Scalar max, Index num_cells) {
    ASSERT(num_cells >= 1, "Number of cells must be at least 1");
    ASSERT(min < max, "Min must be smaller than max");
    const Scalar delta = (max - min) / static_cast<Scalar>(num_cells);
    return RegularIntervalPartition(num_cells, min, max, delta);
  }

  // Creates a regular partition from lower to upper bound with approximately and at least the
  // given cell size.
  static RegularIntervalPartition FromDelta(Scalar min, Scalar max, Scalar delta) {
    ASSERT(delta > 0, "Delta must be greater than 0");
    ASSERT(min < max, "Min must be smaller than max");
    const Index num_cells = static_cast<Index>(std::ceil((max - min) / delta));
    return RegularIntervalPartition(num_cells, min, max, delta);
  }

  // Default constructor creates a single cell unit interval
  RegularIntervalPartition()
  : RegularIntervalPartition(1, Scalar(0), Scalar(1), Scalar(1)) {}

  Index num_cells() const { return num_cells_; }
  Index num_samples() const { return num_cells_ + 1; }
  Scalar min() const { return min_; }
  Scalar max() const { return max_; }
  Scalar delta() const { return regular_partition_.cell_size(); }

  Index toNearest(Scalar value) const {
    return regular_partition_.toNearest(value - min_);
  }
  Index toLinear(Scalar value, Scalar& remainder) const {
    return regular_partition_.toLinear(value - min_, remainder);
  }

  // Computes the mid point value for a cell (no bound checks)
  Scalar cellCenter(Index cell) const {
    return min_ + regular_partition_.cellCenter(cell);
  }
  // Computes the lower bound for a cell (no bound checks)
  Scalar cellLower(Index cell) const {
    return min_ + regular_partition_.cellLower(cell);
  }
  // Computes the upper bound for a cell (no bound checks)
  Scalar cellUpper(Index cell) const {
    return min_ + regular_partition_.cellUpper(cell);
  }

  // Checks if a cell coordinate is inside the partition
  bool inRange(Index cell) const {
    return 0 <= cell && cell < num_cells_;
  }

 private:
  RegularIntervalPartition(Index num_cells, Scalar min, Scalar max, Scalar delta)
  : num_cells_(num_cells), min_(min), max_(max), regular_partition_(delta) { }

  Index num_cells_;
  Scalar min_, max_;
  RegularPartition<Scalar, Index> regular_partition_;
};

}  // namespace partitions
}  // namespace isaac
