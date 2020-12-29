/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <tuple>

#include "engine/core/math/types.hpp"
#include "packages/math/gems/partitions/regular_partition.hpp"

namespace isaac {
namespace partitions {

namespace plane_details {

// A policy which stores a different partition per space dimension
template <typename... Partitions>
class TuplePartitionPolicy {
 public:
  using TupleType = std::tuple<Partitions...>;

  template <int Index> auto& partition() { return std::get<Index>(partitions_); }
  template <int Index> const auto& partition() const { return std::get<Index>(partitions_); }

 private:
  TupleType partitions_;
};

// A policy which uses the same policy for all space dimension
template <typename Partition, size_t Dimension>
class UniformPartitionPolicy {
 public:
  auto& partition() { return partition_; }
  const auto& partition() const { return partition_; }
  template <int Index> const auto& partition() const { return partition_; }

 private:
  Partition partition_;
};

// A partition of 2-dimensional space based on partitions defined by the chosen policy.
template <typename Scalar, typename IndexType, typename Policy>
struct Space2PartitionBase : public Policy {
  using Vector2i = Vector2<IndexType>;
  using Vector2k = Vector2<Scalar>;

  // Converts from world coordinates to grid coordinates
  Vector2k toContinuous(const Vector2k& point) const {
    return {this->template partition<0>().toContinuous(point[0]),
            this->template partition<1>().toContinuous(point[1])};
  }

  // Converts from map coordinates to integer cell coordinate
  Vector2i toNearest(const Vector2k& point) const {
    return {this->template partition<0>().toNearest(point[0]),
            this->template partition<1>().toNearest(point[1])};
  }
  // Converts from map coordinates to integer cell coordinate and also returns the fractional part
  Vector2i toLinear(const Vector2k& point, Vector2k& fractional) const {
    return Vector2i{this->template partition<0>().toLinear(point[0], fractional[0]),
                    this->template partition<1>().toLinear(point[1], fractional[1])};
  }

  // Gets map coordinates for the center of a cell
  Vector2k cellCenter(const Vector2i& cell) const {
    return Vector2k{this->template partition<0>().cellCenter(cell[0]),
                    this->template partition<1>().cellCenter(cell[1])};
  }

  // The dimensions of a cell, i.e. the side lengths of the cell area
  Vector2k cellDimensions() const {
    return Vector2k{this->template partition<0>().cell_size(),
                    this->template partition<1>().cell_size()};
  }

  // Measures the distance between two grid coordinates
  Scalar cartesianDistance(const Vector2i& cell_1, const Vector2i& cell_2) const {
    return (cellCenter(cell_1) - cellCenter(cell_2)).norm();
  }
};

}  // namespace plane_details

// A partition for two-dimensional space
template <typename Scalar, typename Index,
          // partition for the first dimension
          template<class, class> class Partition1,
          // partition for the second dimension
          template<class, class> class Partition2>
using PlanePartition = plane_details::Space2PartitionBase<Scalar, Index,
    plane_details::TuplePartitionPolicy<Partition1<Scalar, Index>,
                                        Partition2<Scalar, Index>>>;

// A plane partition based on different regular partitions for X and Y axes.
template <typename Scalar, typename Index = int>
using RegularPlanePartition = PlanePartition<Scalar, Index, RegularPartition, RegularPartition>;

// A plane partition using the same regular partition for both X and Y axes.
template <typename Scalar, typename Index = int>
using UniformRegularPlanePartition = plane_details::Space2PartitionBase<Scalar, Index,
    plane_details::UniformPartitionPolicy<RegularPartition<Scalar, Index>, 2>>;

}  // namespace partitions
}  // namespace isaac
