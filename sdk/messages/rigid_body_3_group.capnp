#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xa6dcff2fa9085703;

using import "math.capnp".Pose3dProto;
using import "math.capnp".Vector3dProto;

# A rigid body in 3D
struct RigidBody3Proto {
  # The pose of the rigid body with respect to its reference coordinate frame
  refTBody @0: Pose3dProto;
  # The linear velocity in the reference coordinate frame
  linearVelocity @1: Vector3dProto;
  # The angular velocity as a Rodrigues vector in the reference coordinate frame. This means
  # the vector's direction is the axis of rotation and its length is the angular speed around
  # that axis.
  angularVelocity @2: Vector3dProto;
  # The linear acceleration
  linearAcceleration @3: Vector3dProto;
  # The angular acceleration
  angularAcceleration @4: Vector3dProto;
  # The relative scales in x, y, z axis in the body frame; (1.0, 1.0, 1.0) represents the original
  # scale.
  scales @5: Vector3dProto;
}

# A list of rigid bodies in a certain coordinate frame
struct RigidBody3GroupProto {
  # List of rigid bodies
  bodies @0: List(RigidBody3Proto);
  # Optional names for rigid bodies
  names @1: List(Text);
  # Name of coordinate system reference frame
  referenceFrame @2: Text;
}
