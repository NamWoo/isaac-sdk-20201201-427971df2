/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Helper macro: if error, return with error code
#define RETURN_ON_ERROR(error_code)            \
  do {                                         \
    if ((error_code) != isaac_error_success) { \
      return error_code;                       \
    }                                          \
  } while (0)

// Enumerates potential errors for IsaacSDK
typedef enum isaac_error {
  // The operation completed successfully.
  isaac_error_success = 0,
  // An undefined error occurred.
  isaac_error_unknown = -1,
  // An invalid parameter was passed to a function.
  isaac_error_invalid_parameter = -2,
  // Handle is not valid.
  isaac_error_invalid_handle = -3,
  // Invalid buffer size.
  isaac_error_invalid_buffer_size = -4,
  // An operation which requires a node was executed but the node did not exist.
  isaac_error_node_not_found = -5,
  // Unable to find a message ledger for the node
  isaac_error_message_ledger_not_found = -6,
  // Unable to find message queue
  isaac_error_message_queue_not_found = -20,
  // Erro accessing pose between lhs and rhs on the pose tree
  isaac_error_pose = -7,
  // Parameter could not be found in config
  isaac_error_parameter_not_found = -8,
  // Invalid memory type specified
  isaac_error_unknown_memory_type = -9,
  // unable to find a buffer
  isaac_error_buffer_not_found = -10,
  // unable to find a message
  isaac_error_message_not_found = -11,
  // A messages was read but there was no message available.
  isaac_error_no_message_available = -12,
  // Cannot modify a received message
  isaac_error_cannot_modify_received_message = -13,
  // Bad allocation, able to allocate memory
  isaac_error_bad_allocation = -14,
  // All handles have been exhausted
  isaac_error_no_handles_available = -15,
  // this function is deprecated and should not be called
  isaac_error_deprecated = -16,
  // this function will be implemented in the future
  isaac_error_future = -17,
  // invalid message
  isaac_error_invalid_message = -18,
  // Data was not read because the target buffer was null or not big enough to hold all data.
  isaac_error_data_not_read = -19
} isaac_error_t;

// Convert an error code into a human readable error message.
const char* const isaac_get_error_message(isaac_error_t err);

#ifdef __cplusplus
}
#endif
