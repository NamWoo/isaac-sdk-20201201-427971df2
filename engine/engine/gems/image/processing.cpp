/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "processing.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/algorithm.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/gems/image/utils.hpp"

namespace isaac {

template <typename K, typename Container>
void BlurImage(ImageBase<K, 1, Container>& img, int kernel_size) {
  std::vector<K> kernel(2 * kernel_size + 1);
  const K sigma = K(4) / (kernel_size * kernel_size);
  for (int idx = -kernel_size; idx <= kernel_size; idx++) {
    kernel[idx + kernel_size] = std::exp(-idx * idx * sigma);
  }
  Image1d tmp(img.rows(), img.cols());
  for (int row = 0; row < img.rows(); row++) {
    for (int col = 0; col < img.cols(); col++) {
      K sum = K(0);
      K tot = K(0);
      for (int idx = -kernel_size; idx <= kernel_size; idx++) {
        if (idx + row < img.rows() && idx + row >= 0) {
          sum += img(row + idx, col) * kernel[idx + kernel_size];
          tot += kernel[idx + kernel_size];
        }
      }
      tmp(row, col) = sum / tot;
    }
  }
  for (int row = 0; row < img.rows(); row++) {
    for (int col = 0; col < img.cols(); col++) {
      K sum = K(0);
      K tot = K(0);
      for (int idx = -kernel_size; idx <= kernel_size; idx++) {
        if (idx + col < img.cols() && idx + col >= 0) {
          sum += tmp(row, col + idx) * kernel[idx + kernel_size];
          tot += kernel[idx + kernel_size];
        }
      }
      img(row, col) = sum / tot;
    }
  }
}

template <int N>
Image3ub ShrinkSmooth(ImageConstView3ub img) {
  Image3ub out(img.rows() / N, img.cols() / N);
  for (int row = 0; row < out.rows(); row++) {
    for (int col = 0; col < out.cols(); col++) {
      std::array<float, 3> means{{0.0f, 0.0f, 0.0f}};
      for (int rr = 0; rr < N; rr++) {
        for (int cc = 0; cc < N; cc++) {
          for (int k = 0; k < 3; k++) {
            means[k] += static_cast<float>(img(N * row + rr, N * col + cc)[k]);
          }
        }
      }
      for (int k = 0; k < 3; k++) {
        out(row, col)[k] = means[k] / static_cast<float>(N * N);
      }
    }
  }
  return out;
}

// The kernel will not completely overlap with the input image near the image boundaries.
// Specifically the top S/2 rows, bottom S/2 rows, left S/2 columns and right S/2 columns.
// The outside bounds check need not be done in the central region where the kernel is
// guaranteed to overlap. We therefore handle each of these regions separately, so that we do the
// out of bounds check only when necessary.
// We use mirroring when the boundary doesn't overlap with the kernel
template <typename K, int N, typename Container, typename T, int S>
void Convolve2DSeparableKernel(const ImageBase<K, N, Container>& img, const Vector<T, S>& ky,
                               const Vector<T, S>& kx, uint32_t row_delta, uint32_t col_delta,
                               Image<K, N>& dest_img) {
  // Some sanity checks
  ASSERT(img.rows() > 0, "Incorrect input image size");
  ASSERT(img.cols() > 0, "Incorrect input image size");
  ASSERT(row_delta >= 1, "row_delta should be >= 1");
  ASSERT(col_delta >= 1, "col_delta should be >= 1");
  ASSERT(img.rows() > S, "Kernel cannot be bigger than the image");
  ASSERT(img.cols() > S, "Kernel cannot be bigger than the image");
  static_assert(S > 1, "S cannot be negative");
  static_assert(S & 0x1, "S cannot be even");

  const int input_rows = img.rows();
  const int input_cols = img.cols();

  Image<T, N> temp_img(input_rows, input_cols);
  FillElementsWithZero(temp_img);
  // output size is ceil of input size / delta
  const int output_rows = (input_rows + row_delta - 1) / row_delta;
  const int output_cols = (input_cols + col_delta - 1) / col_delta;
  const int S_by_2 = static_cast<int>(S) / 2;

  // Apply ky
  for (int row = 0; row < input_rows; ++row) {
    for (int col = 0; col < input_cols; ++col) {
      T* out_pix = temp_img.row_pointer(row) + col * N;
      for (int k = 0; k < S; ++k) {
        int src_row = row + k > S_by_2 ? row + k - S_by_2 : S_by_2 - (row + k);
        if (src_row >= input_rows) {
          src_row = 2 * input_rows - src_row - 1;  // mirror
        }
        const K* input_pix = img.row_pointer(src_row) + col * N;
        for (int ch = 0; ch < N; ++ch) {
          out_pix[ch] += ky[k] * static_cast<T>(input_pix[ch]);
        }
      }
    }
  }

  dest_img.resize(output_rows, output_cols);
  std::array<T, N> pix;
  // Apply kx
  for (int row = 0; row < input_rows; row += row_delta) {
    int output_row = row / row_delta;
    for (int col = 0; col < input_cols; col += col_delta) {
      int output_col = col / col_delta;
      std::fill(pix.begin(), pix.end(), 0.0f);
      for (int k = 0; k < S; ++k) {
        int src_col = col + k > S_by_2 ? col + k - S_by_2 : S_by_2 - (col + k);
        if (src_col >= input_cols) {
          src_col = 2 * input_cols - src_col - 1;  // mirror
        }
        const T* input_pix = temp_img.row_pointer(row) + src_col * N;
        for (int ch = 0; ch < N; ++ch) {
          pix[ch] += kx[k] * input_pix[ch];
        }
      }
      std::transform(pix.begin(), pix.end(), dest_img.row_pointer(output_row) + output_col * N,
                     [](const T& v) { return static_cast<K>(v); });
    }
  }
}

template <typename K, int N, typename Container>
void DownsampleWithAspectRatio(const ImageBase<K, N, Container>& img, const Vector2i& output_size,
                               Image<K, N>& dest_img, K pad_value) {
  Image<K, N> temp_image, resized_image;
  ASSERT((output_size.array() > 1).all(), "new size should be greater than 1x1");
  ASSERT((output_size.array() < img.dimensions().array()).all(),
         "new size should be smaller than input");
  const int output_rows = output_size[0];
  const int output_cols = output_size[1];

  // Computing the aspect ratio
  const double downsample_factor = std::min(static_cast<double>(output_rows) / img.rows(),
                                            static_cast<double>(output_cols) / img.cols());
  const int resized_rows = static_cast<int>(img.rows() * downsample_factor);
  const int resized_cols = static_cast<int>(img.cols() * downsample_factor);

  // Downsample based on aspect ratio
  Downsample(img, Vector2i{resized_rows, resized_cols}, temp_image);

  // Setting border of image to padding value
  dest_img.resize(output_size);
  FillElements(dest_img, pad_value);

  const int row_padding = (output_rows - resized_rows) / 2;
  const int col_padding = (output_cols - resized_cols) / 2;

  // Copying the resized buffer to letterbox image
  for (int y = 0; y < resized_rows; y++) {
    auto source_row = temp_image.row_pointer(y);
    auto target_row = dest_img.row_pointer(y + row_padding) + col_padding * N;
    std::copy(source_row, source_row + resized_cols * N, target_row);
  }
}

template <typename K, int N, typename Container>
void Downsample(const ImageBase<K, N, Container>& img, const Vector2i& output_size,
                Image<K, N>& dest_img) {
  ASSERT((output_size.array() > 1).all(), "new size should be greater than 1x1");
  ASSERT((output_size.array() < img.dimensions().array()).all(),
         "new size should be smaller than input");
  const int output_rows = output_size[0];
  const int output_cols = output_size[1];

  // Separate the blur kernel
  // 1  4 6  4  1
  // 4 16 24 16 4
  // 6 24 36 24 4
  // 4 16 24 16 4
  // 1  4 6  4  1
  // into two vectors and divide them by 16 so that the resulting kernel has a unit mean
  Vector<float, 5> ky;
  ky << 1.0f, 4.0f, 6.0f, 4.0f, 1.0f;
  ky /= 16.0f;
  Vector<float, 5>& kx = ky;
  Image<K, N> temp_img1(img.rows(), img.cols());
  Copy(img, temp_img1);
  Image<K, N> temp_img2;
  int temp_rows = (temp_img1.rows() + 1) / 2;
  int temp_cols = (temp_img1.cols() + 1) / 2;

  while (temp_rows >= output_rows && temp_cols >= output_cols) {
    // Blur and sample
    Convolve2DSeparableKernel(temp_img1, ky, kx, 2, 2, temp_img2);
    std::swap(temp_img1, temp_img2);
    temp_rows = (temp_img1.rows() + 1) / 2;
    temp_cols = (temp_img1.cols() + 1) / 2;
  }

  // We now have a pyramid image closest to the target resolution in temp_img1
  // we can safely do bilinear interpolation now.
  if (output_rows < temp_img1.rows() || output_cols < temp_img1.cols()) {
    temp_img2.resize(output_rows, output_cols);
    ASSERT(!temp_img2.empty(), "Unexpected empty image");
    const double inc_x = static_cast<double>(temp_img1.cols()) / temp_img2.cols();
    const double inc_y = static_cast<double>(temp_img1.rows()) / temp_img2.rows();
    double src_y = -0.5 + 0.5 * inc_y;
    for (int row = 0; row < output_rows; ++row, src_y += inc_y) {
      double src_x = -0.5 + 0.5 * inc_x;
      for (int col = 0; col < output_cols; ++col, src_x += inc_x) {
        const int x0 = FloorToInt(src_x);
        const int y0 = FloorToInt(src_y);
        const int x1 = std::min(x0 + 1, temp_img1.cols() - 1);
        const int y1 = std::min(y0 + 1, temp_img1.rows() - 1);
        const double x = src_x - static_cast<double>(x0);
        const double y = src_y - static_cast<double>(y0);
        const double xy = x * y;
        // this can be parallelized in Eigen
        K* dest_itr = temp_img2.row_pointer(row) + col * N;
        const K* srcs_row_y0 = temp_img1.row_pointer(y0);
        const K* srcs_row_y1 = temp_img1.row_pointer(y1);
        const int offset_x0 = x0 * N;
        const int offset_x1 = x1 * N;
        const K* f00_itr = srcs_row_y0 + offset_x0;
        const K* f01_itr = srcs_row_y0 + offset_x1;
        const K* f10_itr = srcs_row_y1 + offset_x0;
        const K* f11_itr = srcs_row_y1 + offset_x1;
        for (int ch = 0; ch < N; ++ch) {
          *dest_itr++ = *f00_itr++ * (1.0 - x - y + xy) + *f01_itr++ * (x - xy) +
                        *f10_itr++ * (y - xy) + *f11_itr++ * xy;
        }
      }
    }
    std::swap(temp_img1, temp_img2);
  }

  // Copy the result to destination
  dest_img.resize(output_rows, output_cols);
  Copy(temp_img1, dest_img);
}

template <typename K, int N, typename Container>
bool ImageCorrelation(const ImageBase<K, N, Container>& image_a,
                      const ImageBase<K, N, Container>& image_b, double& correlation) {
  if ((image_a.rows() != image_b.rows()) || (image_a.cols() != image_b.cols())) {
    correlation = 0.0;
    return false;
  }

  double mean[2][N];              // The mean of the samples per channel per image
  double scm[2][N];               // The second central moment of the samples per channel per image
  double scmm[N];                 // The second central mixed moment of the samples per channel
  double channel_correlation[N];  // The correlation between each channel
  for (int channel = 0; channel < N; ++channel) {
    for (int image = 0; image < 2; ++image) {
      mean[image][channel] = 0.0;
      scm[image][channel] = 0.0;
    }
    scmm[channel] = 0.0;
    channel_correlation[channel] = 0.0;
  }

  const int rows = image_a.rows();
  const int cols = image_a.cols();

  int num_samples = 0;  // The number of the samples accumulated
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      num_samples++;
      for (int channel = 0; channel < N; ++channel) {
        double a = static_cast<double>(image_a(row, col)[channel]);
        double b = static_cast<double>(image_b(row, col)[channel]);
        // Compute image channel statistics using the recurrence definifitions
        // This can be more stable for very large numbers, but more importantly
        // means it is only necessary to do one pass through the image.
        const double inv_num_samples = 1.0 / static_cast<double>(num_samples);
        const double sweep = 1.0 - inv_num_samples;
        const double delta_a = a - mean[0][channel];
        const double delta_b = b - mean[1][channel];
        scm[0][channel] += delta_a * delta_a * sweep;
        scm[1][channel] += delta_b * delta_b * sweep;
        scmm[channel] += delta_a * delta_b * sweep;
        mean[0][channel] += delta_a * inv_num_samples;
        mean[1][channel] += delta_b * inv_num_samples;
      }
    }
  }

  // Compute the per channel correlation;
  // There are a number of places where this calculation could go awry due
  // to divide by zero type errors. The final output will be checked with
  // std::isfinite to prevent leaking NaN or inf values.
  // This will not fix the potential numerical accuracy surrounding images
  // with extremely small but non zero variances.
  const double inv_num_samples = 1.0 / (num_samples - 1);
  for (int channel = 0; channel < N; ++channel) {
    const double stddev_a = std::sqrt(scm[0][channel] * inv_num_samples);
    const double stddev_b = std::sqrt(scm[1][channel] * inv_num_samples);
    const double covar_ab = scmm[channel] * inv_num_samples;
    channel_correlation[channel] = covar_ab / (stddev_a * stddev_b);
  }

  // Fuse the channel correlations into a single output value.
  // Uses the Fisher z-transform to sum the per channel values.
  correlation = 0.0;
  for (int channel = 0; channel < N; ++channel) {
    correlation += std::atanh(channel_correlation[channel]);
  }
  correlation /= N;
  correlation = std::tanh(correlation);

  // Check to see if the correlation is a valid number.
  if (std::isfinite(correlation)) {
    return true;
  }
  correlation = 0.0;
  return false;
}

// -------------------------------------------------------------------------------------------------

template void BlurImage(Image1d& img, int kernel_size);

template Image3ub ShrinkSmooth<2>(ImageConstView3ub img);
template Image3ub ShrinkSmooth<4>(ImageConstView3ub img);

template void Downsample(const Image3ub& img, const Vector2i& output_size, Image3ub& dest_img);
template void Downsample(const ImageView3ub& img, const Vector2i& output_size, Image3ub& dest_img);
template void Downsample(const ImageConstView3ub& img, const Vector2i& output_size,
                         Image3ub& dest_img);
template void Downsample(const Image1ub& img, const Vector2i& output_size, Image1ub& dest_img);
template void Downsample(const ImageView1ub& img, const Vector2i& output_size, Image1ub& dest_img);
template void Downsample(const ImageConstView1ub& img, const Vector2i& output_size,
                         Image1ub& dest_img);

template void DownsampleWithAspectRatio(const Image3ub& img, const Vector2i& output_size,
                                        Image3ub& dest_img, uint8_t pad_value);
template void DownsampleWithAspectRatio(const ImageView3ub& img, const Vector2i& output_size,
                                        Image3ub& dest_img, uint8_t pad_value);
template void DownsampleWithAspectRatio(const ImageConstView3ub& img, const Vector2i& output_size,
                                        Image3ub& dest_img, uint8_t pad_value);
template void DownsampleWithAspectRatio(const Image1ub& img, const Vector2i& output_size,
                                        Image1ub& dest_img, uint8_t pad_value);
template void DownsampleWithAspectRatio(const ImageView1ub& img, const Vector2i& output_size,
                                        Image1ub& dest_img, uint8_t pad_value);
template void DownsampleWithAspectRatio(const ImageConstView1ub& img, const Vector2i& output_size,
                                        Image1ub& dest_img, uint8_t pad_value);

template bool ImageCorrelation(const ImageConstView3ub& image_a, const ImageConstView3ub& image_b,
                               double& correlation);

}  // namespace isaac
