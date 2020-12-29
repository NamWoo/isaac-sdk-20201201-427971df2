/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "engine/gems/cuda_utils/stride_pointer.hpp"

namespace isaac {

// Transposes a rank-3 float32 tensor from (row, cols, channels) to (channels, rows, cols) storage.
void Transpose201(StridePointer<const float> input, StridePointer<float> output,
                  int rows, int cols, int channels);

}  // namespace isaac
