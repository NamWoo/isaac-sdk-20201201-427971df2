#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x9b0da06af52fb680;

using import "math.capnp".Vector2dProto;

# Messages published from a gamepad controller.
struct JoystickStateProto {
  # State of gamepad axes
  axes @0: List(Vector2dProto);
  # State of gamepad buttons
  buttons @1: List(Bool);
}
