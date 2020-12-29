#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xf5031e6b6ad6bb47;

using import "math.capnp".Vector2iProto;

# The information of a lattice coordinate system: it contains the name of frame registered in the
# PoseTree and the size of the cells.
struct LatticeProto {
  # The name of frame as registered in the PoseTree
  frame @0: Text;
  # The size of a grid cell in meters
  cellSize @1: Float64;
  # The dimensions of the lattice grid.
  dimensions @2: Vector2iProto;
}
