/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <stdint.h>
// Segway specific crc calculation

namespace isaac {
namespace serialization {

#define CRC_ADJUSTMENT 0xA001
#define CRC_TABLE_SIZE 256
#define INITIAL_CRC (0)

// The CRC table
static uint16_t crc_table[CRC_TABLE_SIZE];
// Private function prototypes
static uint16_t compute_crc_table_value(uint16_t the_byte);

// Initialize the crc table
inline void crc16_initialize(void) {
  uint16_t byte;
  for (byte = 0; byte < CRC_TABLE_SIZE; byte++) {
    crc_table[byte] = compute_crc_table_value(byte);
  }
}

// This computes an updated CRC 16 given the current val the CRC 16 and a new data byte.
inline uint16_t crc16_calculate_crc_16(uint16_t old_crc, uint8_t new_byte) {
  uint16_t temp;
  uint16_t new_crc;
  temp = old_crc ^ new_byte;
  new_crc = (old_crc >> 8) ^ crc_table[temp & 0x00FF];
  return (new_crc);
}

// This function computes the CRC-16 value for the passed in
// buffer. The newly computed CRC is saved into the last
// 2 spots in the byte buffer.
inline void crc16_compute_byte_buffer_crc(uint8_t* byte_buffer, uint32_t bytes_in_buffer) {
  uint32_t count;
  uint32_t crc_index = bytes_in_buffer - 2;
  uint16_t new_crc = INITIAL_CRC;
  // We'll loop through each word of the message and update
  // the CRC. Start with the value chosen for CRC initialization.
  for (count = 0; count < crc_index; count++) {
    // Now we'll send each byte to the CRC calculation.
    new_crc = crc16_calculate_crc_16(new_crc, byte_buffer[count]);
  }

  // The new CRC is saved in the last word.
  byte_buffer[crc_index] = (uint8_t)((new_crc & 0xFF00) >> 8);
  byte_buffer[crc_index+1] = (uint8_t)(new_crc & 0x00FF);
}


// This function computes the CRC-16 value for the passed in
// buffer. This new CRC is compared to the last value stored
// in the buffer (which is assumed to be the CRC-16 for the
// buffer).
inline bool crc16_byte_buffer_crc_is_valid(uint8_t* byte_buffer, uint32_t bytes_in_buffer) {
  uint32_t count;
  uint32_t crc_index = bytes_in_buffer - 2;
  uint16_t new_crc = INITIAL_CRC;
  uint16_t received_crc = INITIAL_CRC;
  bool success;

  // We'll loop through each word of the message and update
  // the CRC. Start with the value chosen for CRC initialization.
  for (count = 0; count < crc_index; count++) {
    new_crc = crc16_calculate_crc_16(new_crc, byte_buffer[count]);
  }
  // The new CRC is checked against that stored in the buffer.
  received_crc = ((byte_buffer[crc_index] << 8) & 0xFF00);
  received_crc |= (byte_buffer[crc_index+1] & 0x00FF);

  if (received_crc == new_crc) {
    success = true;
  } else {
    success = false;
  }
  return (success);
}

// Computes the table value for a given byte
inline uint16_t compute_crc_table_value(uint16_t the_byte) {
  uint16_t j;
  uint16_t k;
  uint16_t table_value;
  k = the_byte;
  table_value = 0;
  for (j = 0; j < 8; j++) {
    if (((table_value ^ k) & 0x0001) == 0x0001) {
      table_value = (table_value >> 1) ^ CRC_ADJUSTMENT;
    } else {
      table_value >>= 1;
    }
    k >>= 1;
  }
  return (table_value);
}

}  // namespace serialization
}  // namespace isaac
