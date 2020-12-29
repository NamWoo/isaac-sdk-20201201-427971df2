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
#include "engine/core/math/types.hpp"

namespace isaac {

// Applies a gaussian blur to an image.
template <typename K, typename Container>
void BlurImage(ImageBase<K, 1, Container>& img, int kernel_size);

// Shrinks the size of an image by the given factor and gives the mean of each pixel.
template <int N>
Image3ub ShrinkSmooth(ImageConstView3ub img);

// Downsamples the image to new size (rows x cols).
// In order to avoid downsampling artifacts, this method will downsample the image one pyramid level
// at a time until we reach a pyramid level that is close to the target size. When we go down one
// pyramid level, we first (gaussian) blur the image using a 5x5 kernel and then sample the colors.
// Once we reach a pyramid level close to the target size, we use bilinear interpolation to do the
// final resize
// Example: input size = 600x1200 target size = 128x293
// We go from 600x1200 -> 300x600 -> 150x300
// Now we do bilinear interpolation to go from 150x300 -> 128x293
template <typename K, int N, typename Container>
void Downsample(const ImageBase<K, N, Container>& img, const Vector2i& output_size,
                Image<K, N>& dest_img);

// Downsamples the image maintaining the aspect ratio. Set the border of the image to the value
// of padding. For example if input width and height of image are (input_cols, input_rows) and
// target width and height are (output_cols, output_rows). downsample_factor =
// min((output_rows/input_rows), (output_cols/ input_cols))
// Output image width without padding : resized_cols = input_cols *  downsample_factor
// Output image height without padding : resized_rows = input_rows * downsample_factor
// Column Padding =  (output_cols - resize_cols)/2 , Row Padding = (output_rows - resized_rows)/2

template <typename K, int N, typename Container>
void DownsampleWithAspectRatio(const ImageBase<K, N, Container>& img, const Vector2i& output_size,
                               Image<K, N>& dest_img, K pad_value = K(0));

// This function will do 2D convolution on the image (over all channels) using a separable 2D kernel
// The original 2D kernel needs to be separated to two vectors kx and ky, where kx is a row vector
// and ky is a column vector. original kernel k = ky * kx.
// ky and kx need to be split in such a way that the mean of the original kernel is 1, otherwise the
// output will not be correct. For example, a 3x3 kernel {1, 2, 3},{2, 4, 6},{3, 6, 9} can be
// separated as ky = {1/6, 2/6, 3/6} and kx = {1/6, 2/6, 3/6}
// If img is a multi-channel image, then the same kernel gets applied to all the channels.
// The size of the kernel should be known at compile time.
// The row_delta and col_delta parameters represent outer and inner stride (in rows and columns
// respectively) while applying the kernel. If these deltas are 1, then the output is the same size
// as input. If these deltas are 2 then the output is half the size of input, since the kernel gets
// applied on every other row and every other column.
// K -> pixel type
// N -> number of channels
// S -> size of the kernel
template <typename K, int N, typename Container, typename T, int S>
void Convolve2DSeparableKernel(const ImageBase<K, N, Container>& img, const Vector<T, S>& ky,
                               const Vector<T, S>& kx, uint32_t row_delta, uint32_t col_delta,
                               Image<K, N>& dest_img);

// Calculate the correlation (-1.0 -> 1.0) between the two input image using
// Zero Mean Normalized Cross Correlation.  If the correlation calculation fails,
// false will be returned. Failure can occur due to an image size mismatch or
// numerical errors.
template <typename K, int N, typename Container>
bool ImageCorrelation(const ImageBase<K, N, Container>& image_a,
                      const ImageBase<K, N, Container>& image_b, double& correlation);

}  // namespace isaac
