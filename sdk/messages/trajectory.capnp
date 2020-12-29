#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xa383942890584edf;

using import "math.capnp".Vector3dProto;

# A trajectory with three dimensions.
# The trajectory is composed of a time series of positions in reference to a given frame.
# For example, one might consider the trajectory of a moving target.
# Note: The timestamps may be omitted, in which case the trajectory is equivalent to a path.
struct Vector3TrajectoryProto {
  # A list of states as positions or samples.
  states @0 : List(Vector3dProto);
  # An optional list of timestamps in seconds corresponding to the list of states or positions.
  timestamps @1 : List(Float64);
  # The reference frame for this trajectory.
  frame @2 : Text;
}

# A list of trajectories with three dimensions.
# For example, one might consider the multiple trajectories for given bodies.
struct Vector3TrajectoryListProto {
  trajectories @0 : List(Vector3TrajectoryProto);
}
