#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xa226180714769561;

using import "math.capnp".Pose2dProto;
using import "image.capnp".ImageProto;

# Describes the heatmap of probabilities for different units
struct HeatmapProto {
  # Probability for each cell of the grid
  heatmap @0: ImageProto;
  # Name of the frame of the map (as registered in the PoseTree)
  frameName @1: Text;
  # Size of a pixel in meters
  gridCellSize @2: Float32;
}
