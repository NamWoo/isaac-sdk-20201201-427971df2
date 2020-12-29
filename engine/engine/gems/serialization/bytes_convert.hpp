/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

namespace isaac {
namespace serialization {

inline void uint16_t_to_bytes(uint16_t value, uint8_t* result) {
  result[0] = value >> 8;
  result[1] = value;
}

inline void bytes_to_uint16_t(uint8_t value[2], uint16_t* result) {
  *result = (uint16_t) ((value[0] << 8) & 0xFF00) |
                       (value[1] & 0x00FF);
}

inline void uint32_t_to_bytes(uint32_t value, uint8_t* result) {
  result[0] = value >> 24;
  result[1] = value >> 16;
  result[2] = value >> 8;
  result[3] = value;
}

inline void bytes_to_uint32_t(uint8_t value[4], uint32_t* result) {
  *result = (uint32_t) ((value[0] << 24) & 0xFF000000) |
                        ((value[1] << 16) & 0x00FF0000) |
                        ((value[2] << 8) & 0x0000FF00) |
                        (value[3] & 0x000000FF);
}

inline void uint32_to_float(uint32_t value, float* result) {
  memcpy(result, &value, 4);
}

inline void bytes_to_float(uint8_t value[4], float* result) {
  uint32_t integer_value = 0;
  bytes_to_uint32_t(value, &integer_value);
  memcpy(result, &integer_value, 4);
}

inline void float_to_bytes(float value, uint8_t result[4]) {
  uint32_t integer_value = 0;
  memcpy(&integer_value, &value, 4);
  uint32_t_to_bytes(integer_value, result);
}

}  // namespace serialization
}  // namespace isaac
