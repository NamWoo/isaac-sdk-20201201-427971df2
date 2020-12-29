/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "isaac_c_api_error.h"

const char* const isaac_get_error_message(isaac_error_t err) {
  switch (err) {
    case isaac_error_success:
      return "Success";
    case isaac_error_unknown:
      return "Unknown";
    case isaac_error_invalid_parameter:
      return "Invalid Parameter";
    case isaac_error_invalid_handle:
      return "Invalid Handle";
    case isaac_error_invalid_buffer_size:
      return "Invalid Buffer Size";
    case isaac_error_node_not_found:
      return "Node Not Found";
    case isaac_error_message_ledger_not_found:
      return "Message Ledger Not Found";
    case isaac_error_message_queue_not_found:
      return "Message Queue Not Found";
    case isaac_error_pose:
      return "Cannot Access Pose";
    case isaac_error_parameter_not_found:
      return "Parameter Not Found";
    case isaac_error_unknown_memory_type:
      return "Unknown Memory Type";
    case isaac_error_buffer_not_found:
      return "Buffer Not Found";
    case isaac_error_message_not_found:
      return "Message Not Found";
    case isaac_error_no_message_available:
      return "No Message Available";
    case isaac_error_cannot_modify_received_message:
      return "Cannot Modify Received Message";
    case isaac_error_bad_allocation:
      return "Bad Allocation";
    case isaac_error_no_handles_available:
      return "All handles have been exhausted";
    case isaac_error_deprecated:
      return "Deprecated";
    case isaac_error_future:
      return "Future";
    case isaac_error_invalid_message:
      return "Invalid Message";
    case isaac_error_data_not_read:
      return "Data was not read";
  }
  // NOTE: we don't have a default case in our switch statement, so if our compiler warnings are set
  // up correctly then we should never reach this line.
  return "Unknown";
}
