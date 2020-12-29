#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xf4df328b0f397d8c;

using import "math.capnp".Vector3dProto;

# Representation of raw marker info tracked by a motion capture system
struct MarkerListProto {
  # List of markers
  markers @0: List(Marker);

  # A motion captured marker
  struct Marker {
    # Marker name or label, if labeled in mocap system
    name @0: Text;
    # Name of node that this marker belongs to, if any
    # If unlabeled, parent is scene root
    parent @1: Text;
    # Translation of marker relative to the global origin
    worldTMarker @2: Vector3dProto;
    # True if marker is occluded in current frame, false otherwise
    occluded @3: Bool;
  }
}
