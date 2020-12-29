/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_message.hpp"

#include <string>
#include <utility>
#include <vector>

#include "capnp/compat/json.h"
#include "capnp/message.h"
#include "capnp/serialize.h"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/capnp.hpp"
#include "messages/camera.hpp"
#include "messages/composite.capnp.h"
#include "messages/image.hpp"
#include "messages/proto_registry.hpp"
#include "messages/range_scan.capnp.h"
#include "messages/state.capnp.h"
#include "messages/tensor.hpp"
#include "messages/uuid.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace isaac {
namespace alice {

namespace {

std::optional<std::string> ParseMessageAsJson(const MessageBase* message) {
  // Check that the message is not null
  if (message == nullptr) {
    return std::nullopt;
  }

  // Check if the message is a JsonMessage
  if (auto* json_msg = dynamic_cast<const JsonMessage*>(message)) {
    // Processes JsonMessage
    return json_msg->data.dump();
  }

  // Check if the message is a ProtoMessage
  if (auto* proto_msg = dynamic_cast<const ProtoMessageBase*>(message)) {
    ::capnp::ReaderOptions options;
    options.traversalLimitInWords = kj::maxValue;
    ::capnp::SegmentArrayMessageReader reader(proto_msg->segments(), options);
    auto maybe_reader = GetRootReaderByTypeId(proto_msg->proto_id(), reader);
    if (maybe_reader != std::nullopt) {
      capnp::JsonCodec json_codec;
      return (::kj::StringPtr)json_codec.encode(*maybe_reader);
    }
  }

  return std::nullopt;
}

// Tries to get a tensor reader from a compatible message
std::optional<::TensorProto::Reader> TryGetTensorReader(const ProtoMessageBase& proto_message) {
  switch (proto_message.type) {
    case ::capnp::typeId<TensorProto>():
      return proto_message.reader().getRoot<TensorProto>();
    case ::capnp::typeId<StateProto>():
      return proto_message.reader().getRoot<StateProto>().getPack();
    case ::capnp::typeId<RangeScanProto>():
      return proto_message.reader().getRoot<RangeScanProto>().getRanges();
    case ::capnp::typeId<CompositeProto>():
      return proto_message.reader().getRoot<CompositeProto>().getValues();
    default:
      return std::nullopt;
  }
}

// Tries to get an image reader from a compatible message
std::optional<::ImageProto::Reader> TryGetImageReader(const ProtoMessageBase& proto_message) {
  switch (proto_message.type) {
    case ::capnp::typeId<ImageProto>():
      return proto_message.reader().getRoot<ImageProto>();
    case ::capnp::typeId<DepthCameraProto>():
      return proto_message.reader().getRoot<DepthCameraProto>().getDepthImage();
    default:
      return std::nullopt;
  }
}

// Tries to get a tensor from a compatible message
std::optional<CpuUniversalTensorConstView> ParseMessageAsTensor(const MessageBase* message) {
  // Check that the message is a proto message
  if (message == nullptr) {
    return std::nullopt;
  }
  auto* proto_message = dynamic_cast<const ProtoMessageBase*>(message);
  if (proto_message == nullptr) {
    return std::nullopt;
  }

  // Trye reading the message as a tensor
  auto tensor_reader = TryGetTensorReader(*proto_message);
  if (tensor_reader) {
    CpuUniversalTensorConstView tensor;
    if (FromProto(*tensor_reader, proto_message->buffers, tensor)) {
      return tensor;
    } else {
      return std::nullopt;
    }
  }

  // Try reading the message as an image
  auto image_reader = TryGetImageReader(*proto_message);
  if (image_reader) {
    CpuUniversalTensorConstView tensor;
    if (FromProto(*image_reader, proto_message->buffers, tensor)) {
      return tensor;
    } else {
      return std::nullopt;
    }
  }

  return std::nullopt;
}

}  // namespace

PybindMessage::PybindMessage(MessageBasePtr message) : message_(std::move(message)) {}

const std::string& PybindMessage::getUuidString() const {
  // Compute UUID string the first time this function is called
  if (uuid_str_ == std::nullopt) {
    if (message_ != nullptr) {
      uuid_str_ = message_->uuid.str();
    } else {
      uuid_str_ = "";
    }
  }
  return *uuid_str_;
}

int64_t PybindMessage::getAcquisitionTime() const {
  if (message_ == nullptr) {
    return 0;
  }
  return message_->acqtime;
}

int64_t PybindMessage::getPublishTime() const {
  if (message_ == nullptr) {
    return 0;
  }
  return message_->pubtime;
}

uint64_t PybindMessage::getTypeId() const {
  if (message_ == nullptr) {
    return 0;
  }
  return message_->type;
}

pybind11::bytes PybindMessage::getCapnProtoBytes() const {
  if (capnp_proto_bytes_ == std::nullopt) {
    if (const ProtoMessageBase* proto_msg = dynamic_cast<const ProtoMessageBase*>(message_.get())) {
      CpuBuffer proto_buffer;
      isaac::serialization::CapnpSegmentsToFlatArray(proto_msg->segments(), proto_buffer);
      std::string proto_str;
      proto_str.resize(proto_buffer.size());
      std::memcpy(const_cast<char*>(proto_str.data()), proto_buffer.begin(), proto_buffer.size());
      capnp_proto_bytes_ = std::move(proto_str);
    } else {
      capnp_proto_bytes_ = "";
    }
  }
  return pybind11::bytes(*capnp_proto_bytes_);
}

std::vector<CpuBufferConstView> PybindMessage::getBuffers() const {
  if (message_ == nullptr) {
    return {};
  }

  // Gets CPU views on all buffers and return them
  std::vector<CpuBufferConstView> result(message_->buffers.size());
  for (size_t index = 0; index < result.size(); index++) {
    result[index] = message_->buffers[index].const_view<CpuBufferConstView>();
  }
  return result;
}

const std::string& PybindMessage::getJsonString() const {
  // Parse the message the first time this function is called
  if (json_string_ == std::nullopt) {
    // Try to get a JSON representation of the message
    json_string_ = ParseMessageAsJson(message_.get());
    // If it failed use an empty string
    if (json_string_ == std::nullopt) {
      json_string_ = "";
    }
  }
  return *json_string_;
}

CpuUniversalTensorConstView PybindMessage::getTensor() const {
  // Parse the message the first time this function is called
  if (tensor_ == std::nullopt) {
    // Try to parse the message as a tensor
    tensor_ = ParseMessageAsTensor(message_.get());
    // If it failed use an empty tensor
    if (tensor_ == std::nullopt) {
      tensor_ = CpuUniversalTensorConstView();
    }
  }
  return *tensor_;
}

bool PybindMessage::isNull() const {
  return message_ == nullptr;
}

ConstMessageBasePtr PybindMessage::getMessage() {
  return message_;
}

void InitPybindMessage(pybind11::module& m) {
  pybind11::class_<PybindMessage>(m, "PybindMessage")
      .def(pybind11::init())
      .def("get_uuid_string", &PybindMessage::getUuidString)
      .def("get_acquisition_time", &PybindMessage::getAcquisitionTime)
      .def("get_publish_time", &PybindMessage::getPublishTime)
      .def("get_type_id", &PybindMessage::getTypeId)
      .def("get_capn_proto_bytes", &PybindMessage::getCapnProtoBytes)
      .def("get_buffers", &PybindMessage::getBuffers)
      .def("get_json_string", &PybindMessage::getJsonString)
      .def("get_tensor", &PybindMessage::getTensor)
      .def("is_null", &PybindMessage::isNull);
}

}  // namespace alice
}  // namespace isaac
