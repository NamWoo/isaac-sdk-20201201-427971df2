/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

// Handle for interacting with IsaacSDK objects.
typedef int64_t isaac_handle_t;

// Enumerates the type of memory isaac can return.
typedef enum isaac_memory_type {
  isaac_memory_none = 0,
  isaac_memory_host = 1,
  isaac_memory_cuda = 2,
} isaac_memory_t;

// Flags for converting messages before publishing
typedef enum isaac_message_convert_flag {
  // Publishes the message as JsonMessage
  isaac_message_type_json = 0,
  // Converts the data and publishes as ProtoMessage
  isaac_message_type_proto = 1,
} isaac_message_convert_t;

// Isaac buffer which can be attached to messages
typedef struct isaac_buffer_type {
  // Pointer to the buffer data
  const unsigned char* pointer;
  // Length of the buffer
  uint64_t size;
  // Storage mode for the buffer
  isaac_memory_t storage;
} isaac_buffer_t;

// Isaac type for passing a read-only JSON objects
typedef struct isaac_json_type {
  // pointer to the JSON data
  char* data;
  // Length of the JSON buffer
  uint64_t size;
} isaac_json_t;

// Isaac type for passing JSON objects
typedef struct isaac_const_json_type {
  // pointer to the JSON data
  const char* data;
  // Length of the JSON buffer
  uint64_t size;
} isaac_const_json_t;

// Isaac UUID
typedef struct isaac_uuid_type {
  uint64_t upper;
  uint64_t lower;
} isaac_uuid_t;

// Isaac transform. Consists of a translation 3 vector and a quaternion for rotation.
typedef struct isaac_pose_type {
  double px, py, pz;
  double qw, qx, qy, qz;
} isaac_pose_t;

#ifdef __cplusplus
}
#endif
