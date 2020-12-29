/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "base64.hpp"

#include <string>

namespace isaac {
namespace serialization {

namespace {

constexpr char kBase64Chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

const std::array<uint8_t, 54> kBmpFileHeader = {
    'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0,      // NOLINT
    40,0,0,0,                                  // NOLINT
    0,0,0,0, 0,0,0,0,  // cols and rows        // NOLINT
    1,0, 24,0,                                 // NOLINT
    0,0,0,0,  // compression                   // NOLINT
    0,0,0,0,  // size, can be zero for BI_RGB  // NOLINT
    0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};       // NOLINT

inline bool IsBase64(uint8_t c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

inline void Encode3Bytes(const uint8_t* src, char* dest) {
  dest[0] = kBase64Chars[(src[0] & 0xfc) >> 2];
  dest[1] = kBase64Chars[((src[0] & 0x03) << 4) + ((src[1] & 0xf0) >> 4)];
  dest[2] = kBase64Chars[((src[1] & 0x0f) << 2) + ((src[2] & 0xc0) >> 6)];
  dest[3] = kBase64Chars[src[2] & 0x3f];
}

inline void EncodeReverse3Bytes(const uint8_t* src, char* dest) {
  dest[0] = kBase64Chars[(src[2] & 0xfc) >> 2];
  dest[1] = kBase64Chars[((src[2] & 0x03) << 4) + ((src[1] & 0xf0) >> 4)];
  dest[2] = kBase64Chars[((src[1] & 0x0f) << 2) + ((src[0] & 0xc0) >> 6)];
  dest[3] = kBase64Chars[src[0] & 0x3f];
}

inline void EncodeDuplicateBytes(const uint8_t src, char* dest) {
  dest[0] = kBase64Chars[(src & 0xfc) >> 2];
  dest[1] = kBase64Chars[((src & 0x03) << 4) + ((src & 0xf0) >> 4)];
  dest[2] = kBase64Chars[((src & 0x0f) << 2) + ((src & 0xc0) >> 6)];
  dest[3] = kBase64Chars[src & 0x3f];
}

}  // namespace

std::string Base64Encode(const Image1ub& image) {
  const size_t data_extra_cols =  (4 - image.cols() % 4) % 4;
  const size_t cols = image.cols() + data_extra_cols;
  const size_t image_size = 3 * cols * image.rows();
  auto copy = kBmpFileHeader;
  uint8_t* bmpfileheader = copy.data();
  *reinterpret_cast<int*>(bmpfileheader + 2) =  static_cast<int>(54 + image_size);
  *reinterpret_cast<int*>(bmpfileheader + 18) = static_cast<int>(cols);
  *reinterpret_cast<int*>(bmpfileheader + 22) = static_cast<int>(image.rows());
  *reinterpret_cast<int*>(bmpfileheader + 34) = static_cast<int>(image_size);

  std::string ret = Base64Encode(bmpfileheader, 54);
  const size_t header_size = ret.size();
  const size_t final_size = header_size + 4 * (image_size / 3);
  ret.resize(final_size);
  char* dst = const_cast<char*>(ret.data()) + header_size;

  // FIXME The following code must be updated to work with image stride

  const size_t cols3 = cols;
  const size_t next_row = image.cols() + cols3;
  const uint8_t* ptr = image.element_wise_end() - image.cols();

  for (size_t row_len = image.rows(); row_len != 0; row_len--) {
    for (const uint8_t* end = ptr + cols3; ptr != end; ptr++, dst += 4) {
      EncodeDuplicateBytes(*ptr, dst);
    }
    ptr -= next_row;
  }
  return ret;
}

std::string Base64Encode(const Image3ub& image) {
  const size_t data_extra_cols = (4 - image.cols() % 4) % 4;
  const size_t cols = image.cols() + data_extra_cols;
  const size_t image_size = 3 * cols * image.rows();
  auto copy = kBmpFileHeader;
  uint8_t* bmpfileheader = copy.data();
  *reinterpret_cast<int*>(bmpfileheader + 2) =  static_cast<int>(54 + image_size);
  *reinterpret_cast<int*>(bmpfileheader + 18) = static_cast<int>(cols);
  *reinterpret_cast<int*>(bmpfileheader + 22) = static_cast<int>(image.rows());
  *reinterpret_cast<int*>(bmpfileheader + 34) = static_cast<int>(image_size);

  std::string ret = Base64Encode(bmpfileheader, 54);
  const size_t header_size = ret.size();
  const size_t final_size = header_size + 4 * (image_size / 3);
  ret.resize(final_size);
  char* dst = const_cast<char*>(ret.data()) + header_size;

  // FIXME The following code must be updated to work with image stride

  const size_t cols3 = 3 * cols;
  const size_t next_row = 3 * image.cols() + cols3;
  const uint8_t* ptr = image.element_wise_end() - 3 * image.cols();

  for (size_t row_len = image.rows(); row_len != 0; row_len--) {
    for (const uint8_t* end = ptr + cols3; ptr != end; ptr += 3, dst += 4) {
      // Reverse color as BMP is BGR.
      EncodeReverse3Bytes(ptr, dst);
    }
    ptr -= next_row;
  }
  return ret;
}

std::string Base64Encode(const uint8_t* ptr, size_t length) {
  const size_t cycles = length / 3;
  const size_t remainder = length % 3;

  std::string ret;
  ret.resize((cycles + (remainder == 0 ? 0 : 1)) * 4);
  char* dst = const_cast<char*>(ret.data());

  // process 3 bytes and convert to 4 base64 bytes
  const uint8_t* bytes_end = ptr + 3 * cycles;
  for (; ptr != bytes_end; ptr += 3, dst += 4) {
    Encode3Bytes(ptr, dst);
  }

  // handle the remaining 1 or 2 bytes if necessary
  if (remainder != 0) {
    // get the remaining bytes into a buffer of three
    uint8_t buffer_3[3];
    buffer_3[0] = ptr[0];
    buffer_3[1] = (remainder == 1 ? 0 : ptr[1]);
    buffer_3[2] = 0;
    // convert three input bytes to 4 base64 bytes
    char buffer_4[4];
    Encode3Bytes(buffer_3, buffer_4);
    // use 2 or 3 bytes and pad with =
    dst[0] = buffer_4[0];
    dst[1] = buffer_4[1];
    dst[2] = (remainder == 1 ? '=' : buffer_4[2]);
    dst[3] = '=';
  }

  return ret;
}

std::string Base64Decode(const std::string& encoded_string) {
  size_t in_len = encoded_string.size();
  int i = 0;
  int in = 0;
  uint8_t char_array_4[4], char_array_3[3];
  std::string ret;
  ret.reserve((encoded_string.size() * 3) / 4 + 1);

  uint8_t lookup[256];
  for (size_t i = 0; i < sizeof(kBase64Chars); i++) {
    lookup[static_cast<int>(kBase64Chars[i])] = static_cast<uint8_t>(i);
  }

  while (in_len-- && encoded_string[in] != '=' && IsBase64(encoded_string[in])) {
    char_array_4[i++] = encoded_string[in];
    in++;
    if (i == 4) {
      for (i = 0; i <4; i++) {
        char_array_4[i] = lookup[char_array_4[i]];
      }

      char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
      char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
      char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

      for (i = 0; (i < 3); i++) {
        ret += char_array_3[i];
      }
      i = 0;
    }
  }

  if (i) {
    for (int j = i; j < 4; j++) {
      char_array_4[j] = 0;
    }
    for (int j = 0; j < 4; j++) {
      char_array_4[j] = lookup[char_array_4[j]];
    }
    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

    for (int j = 0; (j < i - 1); j++) {
      ret += char_array_3[j];
    }
  }
  return ret;
}

}  // namespace serialization
}  // namespace isaac
