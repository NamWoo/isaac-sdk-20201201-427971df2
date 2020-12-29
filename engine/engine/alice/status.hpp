/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

// The status of a component (and thus a node)
enum class Status {
  SUCCESS = 0,  // the component finished successfully
  FAILURE = 1,  // an error was encountered during start, tick, or stop of the component
  RUNNING = 2,  // the comonent is still running
  INVALID = -1  // an invalid status (should never be set)
};

// Used to make status compatible with JSON serialization. As a result status can be used as a
// configuration parameter.
NLOHMANN_JSON_SERIALIZE_ENUM(Status, {
    {Status::INVALID, "invalid"},
    {Status::SUCCESS, "success"},
    {Status::FAILURE, "failure"},
    {Status::RUNNING, "running"},
});

// Converts a status to a string for example for logging
inline const char* ToString(Status status) {
  switch (status) {
    case Status::SUCCESS: return "SUCCESS";
    case Status::FAILURE: return "FAILURE";
    case Status::RUNNING: return "RUNNING";
    default: case Status::INVALID: return "INVALID";
  }
}

// Combines two stati with a standard meaning as shown in the following matrix:
//     | S F R I
//   --|--------
//   S | S F R I
//   F | F F F I
//   R | R F R I
//   I | I I I I
// with S = success, F = failure, R = running, and I = invalid.
inline Status Combine(Status x, Status y) {
  if (x == Status::INVALID || y == Status::INVALID) {
    return Status::INVALID;
  } else if (x == Status::FAILURE || y == Status::FAILURE) {
    return Status::FAILURE;
  } else if (x == Status::RUNNING || y == Status::RUNNING) {
    return Status::RUNNING;
  } else if (x == Status::SUCCESS && y == Status::SUCCESS) {
    return Status::SUCCESS;
  } else {
    return Status::INVALID;  // in case x or y are not set to a valid value
  }
}

}  // namespace alice
}  // namespace isaac
