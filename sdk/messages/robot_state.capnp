#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xd624b315fdd59313;

using import "math.capnp".Pose2dProto;
using import "math.capnp".Vector3dProto;

# The robot state at that particular instance of time. The proto stores the current position and
# angle, as well as the difference between the previous values of the states (position and angle).
# Also contains the current linear and angular speed, obtained from odometry input
struct RobotStateProto {
  # Current pose of robot in the world coordinate frame
  worldTRobot @0: Pose2dProto;
  # Current speed of body (x, y and angular)
  currentSpeed @1: Vector3dProto;
  # Distance travelled from last updated position
  displacementSinceLastUpdate @2: Vector3dProto;
}
