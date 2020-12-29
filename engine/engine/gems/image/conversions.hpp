/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/image/image.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {

// Convert the image encoded in yuyv to rgb.
// The yuyv format is described here:
// https://www.linuxtv.org/downloads/v4l-dvb-apis-old/V4L2-PIX-FMT-YUYV.html
void ConvertYuyvToRgb(const Image2ub& yuyv, Image3ub& rgb);

// Convert a pixels colorspace from rgb to hsv.
// The rgb values are expected to be in [0, 1].
// The result will have H in [0, 360], S in [0, 255], V in [0, 255].
// https://en.wikipedia.org/wiki/HSL_and_HSV
Pixel3f RgbToHsv(const Pixel3f& rgb);

// Convert a float32 RGBA image to a uint8 RGB image
// Input RGBA pixel values are expected to be in [0, 1].
// Result RGB pixel values will be in [0, 255].
void ConvertRgbaToRgb(ImageConstView4f source, ImageView3ub target);

// Convert a uint8 RGBA image to a uint8 RGB image dropping the alpha channel
void ConvertRgbaToRgb(ImageConstView4ub source, ImageView3ub target);

// Convert a uint8 BGRA image to a uint8 RGB image dropping the alpha channel
void ConvertBgraToRgb(ImageConstView4ub source, ImageView3ub target);

// Convert a uint8 RGB image to a uint8 RGBA image and sets the alpha channel to `alpha`.
void ConvertRgbToRgba(ImageConstView3ub source, ImageView4ub target,
                      uint8_t alpha = 255);

// Converts a uint16_t image to a 1f image using a scale factor
void ConvertUi16ToF32(ImageConstView1ui16 source, ImageView1f target, float scale);

// Converts a 1f image to a uint16_t image using a scale factor
void ConvertF32ToUi16(ImageConstView1f source, ImageView1ui16 target, float scale);

// The normalization method for row, column, channel can be specified by the
// ImageToTensorNormalization enum.
enum class ImageToTensorNormalization {
  kNone = 1,
  kUnit = 2,              // Normalize each pixel intensity into a range [0, 1] or vice versa
  kPositiveNegative = 3,  // Normalize each pixel intensity into a range [-1, 1] or vice versa
  kHalfAndHalf = 4        // Normalize each pixel intensity into a range [-0.5, 0.5] or vice versa
};

// Use strings when serializing ImageToTensorNormalization enum
NLOHMANN_JSON_SERIALIZE_ENUM(ImageToTensorNormalization, {
    {ImageToTensorNormalization::kNone, "None"},
    {ImageToTensorNormalization::kUnit, "Unit"},
    {ImageToTensorNormalization::kPositiveNegative, "PositiveNegative"},
    {ImageToTensorNormalization::kHalfAndHalf, "HalfAndHalf"},
});

// Copies an image into a tensor and normalizes each pixel intensity into a given range.
void ImageToNormalizedTensor(
    ImageConstView3ub rgb_image, TensorView3f tensor,
    ImageToTensorNormalization normalization = ImageToTensorNormalization::kPositiveNegative);

// Converts a 3-channel 8-bit image to a 32-bit floating point tensor.
void ImageToNormalizedTensor(
    CudaImageConstView3ub rgb_image, CudaTensorView3f result,
    ImageToTensorNormalization normalization = ImageToTensorNormalization::kPositiveNegative);

// Copies a single channel image into a tensor and normalizes each pixel intensity
// into a given range.
void ImageToNormalizedTensor(
    ImageConstView1ub single_channel_image, TensorView2f tensor,
    ImageToTensorNormalization normalization = ImageToTensorNormalization::kPositiveNegative);

// Converts a 1-channel 8-bit image to a 32-bit floating point tensor.
void ImageToNormalizedTensor(
    CudaImageConstView1ub single_channel_image, CudaTensorView2f result,
    ImageToTensorNormalization normalization = ImageToTensorNormalization::kPositiveNegative);

// Copy a tensor into an image and normalize each pixel intensity into into a given range.
void NormalizedTensorToImage(
    TensorConstView3f tensor,
    ImageToTensorNormalization normalization, Image3ub& rgb_image);

// Copy a 2D tensor into a single channel image and normalize each pixel intensity into
// into a given range.
void NormalizedTensorToImage(
    TensorConstView2f tensor,
    ImageToTensorNormalization normalization, Image1ub& single_channel_image);

}  // namespace isaac
