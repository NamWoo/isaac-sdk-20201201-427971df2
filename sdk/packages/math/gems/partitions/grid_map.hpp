/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/image/image.hpp"
#include "engine/core/math/pose2.hpp"
#include "packages/math/gems/partitions/image_storage.hpp"
#include "packages/math/gems/partitions/plane_partition.hpp"

namespace isaac {
namespace partitions {

// A 2D grid with dimensions and a 2D pose
// Important coordinate frames:
// * "world" reference frame
// * "map" frame which marks the origin of the map
//   The coordinate mapping between world and map is:
//       p_map -> p_world = map_T_world * p_world.
// * "grid" frame which indicate cells in the map using integer or real coordinates
//   The coordinate mapping between map and grid is:
//       p_cell = (row, col) -> cellsize * (row, col) = p_map
//   To get the map coordinates for the center of a grid cell the value 0.5 is added to row/col.
//   Note that x maps to row and y maps to column. This avoids mirroring the map.
template <typename Cell, typename K = double>
class GridMap {
 public:
  using Pose2k = Pose2<K>;
  using Vector2k = Vector2<K>;

  // The pose of the map in the reference coordinate frame
  const Pose2k& map_T_world() const { return map_T_world_; }
  const Pose2k& world_T_map() const { return world_T_map_; }
  void setWorldTMap(const Pose2k& world_T_map) {
    world_T_map_ = world_T_map;
    map_T_world_ = world_T_map_.inverse();
  }

  // The size of a grid cell (cells are square)
  K cell_size() const {
    return partition.partition().cell_size();
  }
  void setCellSize(K cell_size) {
    partition.partition().setCellSize(cell_size);
  }

  // The size of the map
  Vector2k range_max() const {
    return Vector2k{static_cast<K>(rows()) * cell_size(),
                    static_cast<K>(cols()) * cell_size()};
  }
  // The number of rows and columns in the map
  int rows() const { return data.rows(); }
  int cols() const { return data.cols(); }

  // Checks if a coordinate indicates a valid cell in the map
  bool isInRange(int row, int col) const {
    return data.isInRange(row, col);
  }
  bool isInRange(const Vector2i& cell) const {
    return data.isInRange(cell[0], cell[1]);
  }

  // Gets the cell at the given pixel coordinate (no range check)
  const Cell& at(int row, int col) const {
    return data.at(row, col);
  }
  Cell& at(int row, int col) {
    return data.at(row, col);
  }
  const Cell& at(const Vector2i& cell) const {
    return data.at(cell[0], cell[1]);
  }
  Cell& at(const Vector2i& cell) {
    return data.at(cell[0], cell[1]);
  }

  // Converts from world coordinates to grid coordinates
  Vector2k mapToGrid(const Vector2k& p_map) const {
    return partition.toContinuous(p_map);
  }

  // Converts from map coordinates to integer cell coordinate
  Vector2i mapToCell(const Vector2k& p_map) const {
    return partition.toNearest(p_map);
  }
  // Converts from map coordinates to integer cell coordinate and also returns the fractional part
  Vector2i mapToCell(const Vector2k& p_map, Vector2k& fractional_grid) const {
    return partition.toLinear(p_map, fractional_grid);
  }

  // Gets map coordinates for the center of a cell
  Vector2k cellCenterToMap(const Vector2i& p_grid) const {
    return partition.cellCenter(p_grid);
  }

  // Measures the distance between to grid coordinates
  K measureCellDistance(const Vector2i& p_grid_1, const Vector2i& p_grid_2) const {
    return partition.cartesianDistance(p_grid_1, p_grid_2);
  }

  // Creates a copy
  GridMap<Cell, K> clone() const {
    GridMap<Cell, K> other;
    other.setWorldTMap(this->world_T_map_);
    other.partition = this->partition;
    other.data.image().resize(this->data.image().dimensions());
    Copy(this->data.image(), other.data.image());
    return other;
  }

  // The stored cell data
  ImageStorage<Cell, int> data;

  // The mapping between grid and Cartesian space (without transformation)
  UniformRegularPlanePartition<K, int> partition;

 private:
  Pose2k world_T_map_;
  Pose2k map_T_world_;
};

// A grid map storing 0/1 values for each cell (currently implemented with 8-bit integers)
using BinaryGridMapF = GridMap<uint8_t, float>;

// A grid map storing a 32-bit float for each cell
using FloatGridMapF = GridMap<float, float>;

}  // namespace partitions
}  // namespace isaac
