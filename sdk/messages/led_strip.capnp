#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x8c172d638f8a549a;

using import "math.capnp".Vector3ubProto;

# A message to describe a desired state of a LED display strip
struct LedStripProto {
  # Light strip status. If false, the LED strip is powered off.
  # If true, the LED strip displays the pattern and color
  # defined by the following parameters of this message.
  enabled @0: Bool;

  # Color to display, in RGB format
  color @1: Vector3ubProto;

  # Number of LEDs to skip when displaying this color. The remaining
  # LEDs will be unchanged.
  skip @2: Int32;

  # If a skip value is defined, the offset is the index of LED to start
  # the skip pattern. i.e. To set every other LED
  # starting from the third LED use skip = 2, offset = 3
  offset @3: Int32;
}
