/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "conversions.hpp"

#include <algorithm>

#include "engine/core/math/utils.hpp"
#include "engine/gems/image/color.hpp"
#include "engine/gems/image/cuda/image_to_tensor.cu.hpp"

namespace isaac {

namespace {

// Use a bitshift to quickly divide a number by 256.
uint8_t Div256(int32_t val) {
  return static_cast<uint8_t>(val >> 8);
}

// R = 1.164*(Y' - 16) + 1.596*(Cr - 128)
// G = 1.164*(Y' - 16) - 0.813*(Cr - 128) - 0.391*(Cb - 128)
// B = 1.164*(Y' - 16)                    + 2.018*(Cb - 128)
//
// The coefficients below are the numbers from YCbCr-to-RGB conversion matrix
// converted to 8:8 fixed point numbers.
//
// See: https://en.wikipedia.org/wiki/YCbCr, https://en.wikipedia.org/wiki/YUV
//
void YCbCrToRgb(uint8_t y, uint8_t cb, uint8_t cr, uint8_t* out) {
  const int32_t y_shifted = y - 16;
  const int32_t cr_shifted = cr - 128;
  const int32_t cb_shifted = cb - 128;

  out[0] = Div256(Clamp(298 * y_shifted + 409 * cr_shifted, 0, 0xffff));
  out[1] = Div256(Clamp(298 * y_shifted - 208 * cr_shifted - 100 * cb_shifted, 0, 0xffff));
  out[2] = Div256(Clamp(298 * y_shifted + 516 * cb_shifted, 0, 0xffff));
}

// Converts one image into another image
template <typename K1, int N1, typename Container1, typename K2, int N2, typename Container2,
          typename F>
void ConvertImpl(const ImageBase<K1, N1, Container1>& source, ImageBase<K2, N2, Container2>& target,
                 F op) {
  ASSERT(source.rows() == target.rows(), "row count mismatch: %zd vs. %zd", source.rows(),
         target.rows());
  ASSERT(source.cols() == target.cols(), "col count mismatch: %zd vs. %zd", source.cols(),
         target.cols());
  for (int row = 0; row < source.rows(); row++) {
    const K1* it_source = source.row_pointer(row);
    const K1* it_source_end = it_source + N1 * source.cols();
    K2* it_target = target.row_pointer(row);
    for (; it_source != it_source_end; it_source += N1, it_target += N2) {
      op(it_source, it_target);
    }
  }
}

}  // namespace

void ConvertYuyvToRgb(const Image2ub& yuyv, Image3ub& rgb) {
  ASSERT(yuyv.num_pixels() % 2 == 0, "invalid input image size");
  rgb.resize(yuyv.rows(), yuyv.cols());

  // Each 4 byte block encodes two pixels, so we read 4 bytes at a time.
  const uint32_t* raw_in = reinterpret_cast<const uint32_t*>(yuyv.element_wise_begin());
  const uint32_t* raw_end = reinterpret_cast<const uint32_t*>(yuyv.element_wise_end());
  uint8_t* raw_out = rgb.element_wise_begin();

  // Convert each 2 pixel block.
  for (; raw_in != raw_end; ++raw_in) {
    const uint32_t yuvu_block = *raw_in;
    const uint8_t y0 = yuvu_block & 0xff;
    const uint8_t cb = (yuvu_block >> 8) & 0xff;
    const uint8_t y1 = (yuvu_block >> 16) & 0xff;
    const uint8_t cr = (yuvu_block >> 24) & 0xff;

    // Encode first pixel.
    YCbCrToRgb(y0, cb, cr, raw_out);
    raw_out += 3;

    // Encode second pixel.
    YCbCrToRgb(y1, cb, cr, raw_out);
    raw_out += 3;
  }
}

Pixel3f RgbToHsv(const Pixel3f& rgb) {
  const float max = std::max({rgb[0], rgb[1], rgb[2]});
  const float min = std::min({rgb[0], rgb[1], rgb[2]});
  const float delta = max - min;
  Pixel3f hsv{0.0f, 0.0f, max * 255.0f};
  if (delta == 0.0f) {
    return hsv;
  }
  hsv[1] = (delta / max) * 255.0f;

  // hue
  if (max == rgb[0]) {
    hsv[0] = 60.0f * ((rgb[1] - rgb[2]) / delta);
  } else if (max == rgb[1]) {
    hsv[0] = 60.0f * ((rgb[2] - rgb[0]) / delta) + 120.0f;
  } else {
    hsv[0] = 60.0f * ((rgb[0] - rgb[1]) / delta) + 240.0f;
  }

  if (hsv[0] < 0.0f) hsv[0] += 360.0f;
  return hsv;
}

void ConvertRgbaToRgb(ImageConstView4f source, ImageView3ub target) {
  ConvertImpl(source, target, [](const float* src, uint8_t* dst) {
    dst[0] = static_cast<uint8_t>(src[0] * 255.0f + 0.5f);
    dst[1] = static_cast<uint8_t>(src[1] * 255.0f + 0.5f);
    dst[2] = static_cast<uint8_t>(src[2] * 255.0f + 0.5f);
  });
}

void ConvertRgbaToRgb(ImageConstView4ub source, ImageView3ub target) {
  ConvertImpl(source, target, [](const uint8_t* src, uint8_t* dst) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
  });
}

void ConvertBgraToRgb(ImageConstView4ub source, ImageView3ub target) {
  ConvertImpl(source, target, [](const uint8_t* src, uint8_t* dst) {
    dst[0] = src[2];
    dst[1] = src[1];
    dst[2] = src[0];
  });
}

void ConvertRgbToRgba(ImageConstView3ub source, ImageView4ub target, uint8_t alpha) {
  ConvertImpl(source, target, [alpha](const uint8_t* src, uint8_t* dst) {
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = alpha;
  });
}

void ConvertUi16ToF32(ImageConstView1ui16 source, ImageView1f target, float scale) {
  ConvertImpl(source, target, [scale](const uint16_t* src, float* dst) {
    *dst = static_cast<float>(*src) * scale;
  });
}

void ConvertF32ToUi16(ImageConstView1f source, ImageView1ui16 target, float scale) {
  const float scale_inv = 1.0f / scale;
  ConvertImpl(source, target, [=](const float* src, uint16_t* dst) {
    *dst = static_cast<uint16_t>(Clamp(*src * scale_inv, 0.0f, 65535.0f));
  });
}

void ImageToNormalizedTensor(ImageConstView3ub input, TensorView3f output,
                             ImageToTensorNormalization normalization) {
  const int rows = input.rows();
  const int cols = input.cols();
  ISAAC_ASSERT_EQ(rows, output.dimensions()[0]);
  ISAAC_ASSERT_EQ(cols, output.dimensions()[1]);
  ISAAC_ASSERT_EQ(3, output.dimensions()[2]);

  if (normalization == ImageToTensorNormalization::kPositiveNegative) {
    constexpr float kScale = 2.0f / 255.0f;
    for (int row = 0; row < rows; row++) {
      output.slice(row).matrix() =
          (input.tensor().const_slice(row).matrix().array().cast<float>() * kScale - 1.0f).matrix();
    }
  } else if (normalization == ImageToTensorNormalization::kUnit) {
    constexpr float kScale = 1.0f / 255.0f;
    for (int row = 0; row < rows; row++) {
      output.slice(row).matrix() = input.tensor().const_slice(row).matrix().cast<float>() * kScale;
    }
  } else if (normalization == ImageToTensorNormalization::kNone) {
    for (int row = 0; row < rows; row++) {
      output.slice(row).matrix() = input.tensor().const_slice(row).matrix().cast<float>();
    }
  } else if (normalization == ImageToTensorNormalization::kHalfAndHalf) {
    constexpr float kScale = 1.0f / 255.0f;
    for (int row = 0; row < rows; row++) {
      output.slice(row).matrix() =
          (input.tensor().const_slice(row).matrix().array().cast<float>() * kScale - 0.5f).matrix();
    }
  } else {
    PANIC("Invalid normalization method: %d", normalization);
  }
}

void ImageToNormalizedTensor(CudaImageConstView3ub rgb_image, CudaTensorView3f result,
                             ImageToTensorNormalization normalization) {
  float factor, bias;
  switch (normalization) {
    case ImageToTensorNormalization::kPositiveNegative: factor = 2.0f / 255.0f; bias = -1.0f; break;
    case ImageToTensorNormalization::kUnit:             factor = 1.0f / 255.0f; bias =  0.0f; break;
    case ImageToTensorNormalization::kNone:             factor = 1.0f;          bias =  0.0f; break;
    case ImageToTensorNormalization::kHalfAndHalf:      factor = 1.0f / 255.0f; bias = -0.5f; break;
    default: PANIC("Invalid image normalization mode: %d", normalization); break;
  }

  ISAAC_ASSERT_EQ(rgb_image.rows(), result.dimensions()[0]);
  ISAAC_ASSERT_EQ(rgb_image.cols(), result.dimensions()[1]);
  ISAAC_ASSERT_EQ(3, result.dimensions()[2]);
  ImageToTensor({rgb_image.element_wise_begin(), rgb_image.getStride()},
                {result.element_wise_begin(), rgb_image.cols() * 3 * sizeof(float)},
                rgb_image.rows(), rgb_image.cols(), static_cast<int>(3), factor, bias);
}

void ImageToNormalizedTensor(ImageConstView1ub input, TensorView2f output,
                             ImageToTensorNormalization normalization) {
  ISAAC_ASSERT_EQ(input.rows(), output.dimensions()[0]);
  ISAAC_ASSERT_EQ(input.cols(), output.dimensions()[1]);

  if (normalization == ImageToTensorNormalization::kPositiveNegative) {
    output.matrix() =
        (input.tensor().matrix().cast<float>().array() * (2.0f / 255.0f) - 1.0f).matrix();
  } else if (normalization == ImageToTensorNormalization::kUnit) {
    output.matrix() = input.tensor().matrix().cast<float>() * (1.0f / 255.0f);
  } else if (normalization == ImageToTensorNormalization::kNone) {
    output.matrix() = input.tensor().matrix().cast<float>();
  } else if (normalization == ImageToTensorNormalization::kHalfAndHalf) {
    output.matrix() =
        (input.tensor().matrix().cast<float>().array() * (1.0f / 255.0f) - 0.5f).matrix();
  } else {
    PANIC("Invalid image normalization mode: %d", normalization);
  }
}

void ImageToNormalizedTensor(CudaImageConstView1ub single_channel_image, CudaTensorView2f result,
                             ImageToTensorNormalization normalization) {
  float factor, bias;
  switch (normalization) {
    case ImageToTensorNormalization::kPositiveNegative: factor = 2.0f / 255.0f; bias = -1.0f; break;
    case ImageToTensorNormalization::kUnit:             factor = 1.0f / 255.0f; bias =  0.0f; break;
    case ImageToTensorNormalization::kNone:             factor = 1.0f;          bias =  0.0f; break;
    case ImageToTensorNormalization::kHalfAndHalf:      factor = 1.0f / 255.0f; bias = -0.5f; break;
    default: PANIC("Invalid image normalization mode: %d", normalization); break;
  }

  ASSERT(single_channel_image.rows() == result.dimensions()[0],
         "row count mismatch between image columns %d and tensor columns %d",
         single_channel_image.rows(), result.dimensions()[0]);
  ASSERT(single_channel_image.cols() == result.dimensions()[1],
         "col count mismatch between image columns %d and tensor columns %d",
         single_channel_image.cols(), result.dimensions()[1]);
  ImageToTensor({single_channel_image.element_wise_begin(), single_channel_image.getStride()},
                {result.element_wise_begin(), single_channel_image.cols() * 1 * sizeof(float)},
                single_channel_image.rows(), single_channel_image.cols(), static_cast<int>(1),
                factor, bias);
}

void NormalizedTensorToImage(TensorConstView3f tensor, ImageToTensorNormalization normalization,
                             Image3ub& rgb_image) {
  const int tensor_rows = tensor.dimensions()[0];
  const int tensor_cols = tensor.dimensions()[1];
  rgb_image.resize(tensor_rows, tensor_cols);
  if (normalization == ImageToTensorNormalization::kPositiveNegative) {
    for (int row = 0; row < tensor_rows; row++) {
      rgb_image.tensor().slice(row).matrix() =
          ((tensor.const_slice(row).matrix().array() + 1.0f) * 127.5f).cast<uint8_t>().matrix();
    }
  } else if (normalization == ImageToTensorNormalization::kUnit) {
    for (int row = 0; row < tensor_rows; row++) {
      rgb_image.tensor().slice(row).matrix() =
          (tensor.const_slice(row).matrix() * 255.0f).cast<uint8_t>();
    }
  } else if (normalization == ImageToTensorNormalization::kNone) {
    for (int row = 0; row < tensor_rows; row++) {
      rgb_image.tensor().slice(row).matrix() = tensor.const_slice(row).matrix().cast<uint8_t>();
    }
  } else if (normalization == ImageToTensorNormalization::kHalfAndHalf) {
    for (int row = 0; row < tensor_rows; row++) {
      rgb_image.tensor().slice(row).matrix() =
          ((tensor.const_slice(row).matrix().array() + 0.5f) * 255.0f).cast<uint8_t>().matrix();
    }
  } else {
    PANIC("Invalid normalization method: %d", normalization);
  }
}

void NormalizedTensorToImage(TensorConstView2f tensor, ImageToTensorNormalization normalization,
                             Image1ub& single_channel_image) {
  const int tensor_rows = tensor.dimensions()[0];
  const int tensor_cols = tensor.dimensions()[1];
  single_channel_image.resize(tensor_rows, tensor_cols);
  if (normalization == ImageToTensorNormalization::kPositiveNegative) {
    single_channel_image.tensor().matrix() =
          ((tensor.matrix().array() + 1.0f) * 127.5f).cast<uint8_t>().matrix();
  } else if (normalization == ImageToTensorNormalization::kUnit) {
    single_channel_image.tensor().matrix() = (tensor.matrix() * 255.0f).cast<uint8_t>();
  } else if (normalization == ImageToTensorNormalization::kNone) {
    single_channel_image.tensor().matrix() = tensor.matrix().cast<uint8_t>();
  } else if (normalization == ImageToTensorNormalization::kHalfAndHalf) {
    single_channel_image.tensor().matrix() =
          ((tensor.matrix().array() + 0.5f) * 255.0f).cast<uint8_t>().matrix();
  } else {
    PANIC("Invalid normalization method: %d", normalization);
  }
}

}  // namespace isaac
