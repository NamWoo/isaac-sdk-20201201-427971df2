#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x91f9f82d2f6a05e2;

using import "math.capnp".Pose3dProto;
using import "math.capnp".Vector3dProto;

# A collision event information
struct CollisionProto {
  # The name of this collider
  thisName @0: Text;
  # The name of the other collider
  otherName @1: Text;
  # The pose of this collider with respect to its reference coordinate frame
  thisPose @2: Pose3dProto;
  # The pose of the other collider with respect to the same reference coordinate frame
  otherPose @3: Pose3dProto;
  # contact point of the collision in the reference coordinate frame
  contactPoint @4: Vector3dProto;
  # normal of the contact point in the reference coordinate frame
  contactNormal @5: Vector3dProto;
  # The relative linear velocity between the two colliding objects in the reference coordinate frame
  velocity @6: Vector3dProto;
}