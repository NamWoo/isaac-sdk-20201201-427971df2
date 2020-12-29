/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/message.hpp"
#include "engine/core/buffers/buffer.hpp"
#include "engine/core/optional.hpp"
#include "engine/core/tensor/universal_tensor.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

// Pybind API to provide access to an Isaac SDK message
class PybindMessage {
 public:
  PybindMessage() = default;
  PybindMessage(MessageBasePtr message);
  ~PybindMessage() = default;

  // Returns the UUID of the message as a string
  const std::string& getUuidString() const;

  // Returns the acqtime of the message as a string
  int64_t getAcquisitionTime() const;

  // Returns the pubtime of the message as a string
  int64_t getPublishTime() const;

  // Returns the type ID of the message
  uint64_t getTypeId() const;

  // Returns the bytes of the Capn'proto blob
  pybind11::bytes getCapnProtoBytes() const;

  // Returns a list with all message buffers
  std::vector<CpuBufferConstView> getBuffers() const;

  // Converts the message message to JSON and returns the JSON as a string
  const std::string& getJsonString() const;

  // Interprets the message as a single tensor. Returns a 0-rank tensor if the message can not be
  // interpreted as a tensor.
  CpuUniversalTensorConstView getTensor() const;

  // Returns true is the message is null.
  bool isNull() const;

  ConstMessageBasePtr getMessage();

 private:
  MessageBasePtr message_;
  mutable std::optional<std::string> uuid_str_;
  mutable std::optional<std::string> capnp_proto_bytes_;
  mutable std::optional<CpuUniversalTensorConstView> tensor_;
  mutable std::optional<std::string> json_string_;
};

// Initializes the python module
void InitPybindMessage(pybind11::module& module);

}  // namespace alice
}  // namespace isaac
