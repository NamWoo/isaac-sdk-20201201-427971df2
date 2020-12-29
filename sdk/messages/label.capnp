#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xd7f13dca2731c8f0;

# Mapping of label index to label name for segmentation images
struct LabelProto {
  # A label used to enumerate all labels in the `labels` field below
  struct Label {
    # The integer value stored in the `labelImage`
    index @0: UInt8;
    # The name of the label
    name @1: Text;
  }
  # List of all labels used in labelImage
  labels @0: List(Label);
}