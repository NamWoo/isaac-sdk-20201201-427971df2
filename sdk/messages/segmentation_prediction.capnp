#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x8753deaa690d85b3;

using import "tensor.capnp".TensorProto;

# Segmentation prediction proto stores the probability distribution over classes for each pixel in
# a three dimensional tensor. The probability will add up to one for each pixel.
struct SegmentationPredictionProto {

  # List of all class names. The number of elements must match the third dimension of the tensor.
  classNames @0: List(Text);
  # Tensor with dimensions (rows * cols * classes)
  prediction @1: TensorProto;
}
