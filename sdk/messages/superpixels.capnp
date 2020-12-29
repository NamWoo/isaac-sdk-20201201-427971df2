#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xb89f858c1b503640;

using import "math.capnp".Vector2fProto;
using import "math.capnp".Vector3fProto;
using import "image.capnp".ImageProto;

# A superpixel oversegmentation for an image. This is useful for various image segmentation methods
# or perception algorithms in general.
#
# Each pixel in the image is assigned to a superpixel. For each superpixel information like the
# pixel coordinate or the color are reported. Optionally also 3D data like point and normal are
# reported.
struct SuperpixelsProto {
  # 2D superpixel data based on the color image
  struct Superpixel {
    # The average pixel coordinate of the superpixel stored as (row, col)
    pixel @0: Vector2fProto;
    # The average color of the superpixel stored as RGB in the range [0,1]
    color @1: Vector3fProto;
    # The number of pixels in this superpixel. There might be superpixels with count = 0 which
    # indicates that no pixels were assigned to them. This would also indicate that other superpixel
    # and surflet data is invalid.
    count @2: UInt32;
  }

  # Additional 3D surflet data for each superpixel defining the 3D shape of the superpixel
  struct Surflet {
    # 3D point coordinates of the surflet
    point @0: Vector3fProto;
    # 3D normal of the surflet
    normal @1: Vector3fProto;
  }

  # An image which assignes a superpixel image to every pixel using the superpixel index. This index
  # is identical to the position of the superpixel in the superpixels or surflet lists. Indices are
  # stored as an unsinged 16-bit integer. Pixels which are not assigned to any superpixel are marked
  # with the index 0xffff.
  indices @0: ImageProto;

  # 2D superpixel data for every pixel cluster
  superpixels @1: List(Superpixel);

  # 3D surflet data for every pixel cluster (optional)
  surflets @2: List(Surflet);
}

# Labels for superpixels. Assigns a label to every superpixel in a related SuperpixelsProto
struct SuperpixelLabelsProto {
  labels @0: List(Int32);
}
