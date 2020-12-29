#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xf0e8f25da377ffe4;

using import "math.capnp".Vector2dProto;
using import "math.capnp".Pose3dProto;

# A fiducial is an object placed in the field of view
# of an imaging system for use as a point of reference or a measure
struct FiducialProto {
  # A fiducial can be of type (April Tag, QRCode, Barcode or ARTag)
  enum Type {
    apriltag @0;
    qrcode @1;
    barcode @2;
    artag @3;
  }
  # Enum to identify the type of fiducial represented by the
  # proto instance
  type @0 : Type;

  # Text field that identifies the ID of the fiducial
  # For April Tag, the id is of the format <TagFamily_ID>
  # Ex. If the decoded tag ID is 14 and belongs to TagFamily tag36h11,
  # The id is tag36h11_14
  id @1 : Text;

  # 3D pose of the detected tag from the camera coordinates,
  # consisting of orientation (quaternion) and translation
  # Camera coordinate (X - right, Y - down, Z - outward)
  # Tag coordinate (X - right, Y - down, Z - opposite to tag face direction)
  # Tag coordinates are centered at tag's upper-left corner
  # ie. Pose has identity quaternion and zero translation, when tag is facing the camera and it's
  # upper-left corner is centered at the camera center
  cameraTTag @2 : Pose3dProto;

  # Optional list of keypoints of the detected fiducial, in image space
  keypoints @3 : List(Vector2dProto);
}

# A list of detected ficucials
struct FiducialListProto {
  fiducialList @0 : List(FiducialProto);
}
