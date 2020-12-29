#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x8f695632cc0cec36;

using import "math.capnp".Vector3dProto;
using import "math.capnp".Pose2dProto;
using import "image.capnp".ImageProto;
using import "uuid.capnp".UuidProto;

# A message containing all obstacles around the robot
struct ObstaclesProto {
  # A message containing the information about a spherical obstacle
  struct SphereObstacleProto {
    # The positions of the center in a given frame (see frame below)
    center @0: Vector3dProto;
    # The radius of the obstacle (include the minimal distance we aim to stay away from)
    radius @1: Float64;
    # The coordinate frame the center is located (must match a frame in the PoseTree)
    # deprecated: use robotFrame below instead.
    frame @2: Text;
    # Unique identifier the can be used to track the object
    uuid @3: UuidProto;
    # Time this obstacle was reported (used in the PoseTree)
    time @4: Float64;
  }

  # A distance map describes the distance to the closest obstacle for every pixel.
  struct DistanceMapProto {
    # The distance map as a 1d image. Each pixel contains the metric distance to the closest
    # obstacle
    image @0: ImageProto;
    # The pose of the gridmap relative to the robot
    robotTGridmap @1: Pose2dProto;
    # The size of a grid cell
    gridCellSize @2: Float64;
    # Time this obstacle was reported (used in the PoseTree)
    time @3: Float64;
  }

  # List of spherical obstacles (see SphereObstacleProto for details)
  obstacles @0: List(SphereObstacleProto);

  # List of distance maps (see DistanceMapProto for details)
  distanceMaps @1: List(DistanceMapProto);

  # The coordinate frame the center is located (must match a frame in the PoseTree)
  robotFrame @2: Text;
}
