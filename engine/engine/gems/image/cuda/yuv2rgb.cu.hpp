/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include <cuda_runtime.h>

namespace isaac {

// Converts a YUV420 (encoded as NV21) image bound to texture channels to RGB
void ConvertNv21ToRgb(cudaTextureObject_t y_channel, cudaTextureObject_t uv_channel,
                      unsigned char* rgb_output, unsigned int width, unsigned int height,
                      unsigned int output_pitch);

}  // namespace isaac
