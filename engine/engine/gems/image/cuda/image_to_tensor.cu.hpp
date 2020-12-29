/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "engine/gems/cuda_utils/stride_pointer.hpp"

namespace isaac {

// Converts a 3 or 1-channel 8-bit image to a 32-bit floating point  tensor.
// Tensor data is store in row, column, channel order.
void ImageToTensor(StridePointer<const unsigned char> image, StridePointer<float> result,
                   size_t rows, size_t cols, int channels, float factor, float bias);

}  // namespace isaac

