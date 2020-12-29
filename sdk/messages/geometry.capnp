#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xaa2652a8195b8a54;

using import "math.capnp".Vector2dProto;
using import "math.capnp".Vector3dProto;

# A plane in three-dimensional space
struct PlaneProto {
  normal @0 : Vector3dProto;
  offset @1 : Float64;
}

# A box (cuboid-3) using 64-bit doubles
struct BoxProto {
  # (x, y, z) values of the lower boundaries of the box
  min @0 : Vector3dProto;
  # (x, y, z) values of the upper boundaries of the box
  max @1 : Vector3dProto;
}

# A rectangle using 64-bit doubles
struct RectangleProto {
  # (x, y) values of the lower left corner of the rectangle
  min @0 : Vector2dProto;
  # (x, y) values of the upper right corner of the rectangle
  max @1 : Vector2dProto;
}

# (deprecated) A message describing the location of the ground.
struct GroundPlaneProto {
  # The ground plane.
   plane @0: PlaneProto;
}

# A polyline in 2d represented by a list of points.
struct Polyline2dProto {
  line @0: List(Vector2dProto);
}
