/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "capnp/message.h"
#include "capnp/serialize.h"
#include "engine/core/assert.hpp"
#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

// Base class for messages
class MessageBase {
 public:
  virtual ~MessageBase() = default;

  // The size of the message-specific payload in bytes
  virtual size_t getPayloadSize() const = 0;

  // The size of buffers in this message in bytes
  size_t getBufferSize() const;

  // The approximate total size of this message in bytes
  size_t size() const;

  // Uniquely identifies the messages across systems
  Uuid uuid;

  // A timestamp in nanoseconds relating when the hardware acquired data. For many messages it
  // makes sense to still relate to when the hardware acquired data originally even after multiple
  // processing steps.
  // For example face detections on an image would carry forward the acquisition time of the image
  // itself. This allows other parts of the system to synchronize data which was computed on
  // the same image.
  int64_t acqtime;

  // The time in nanoseconds when this message was published. Publish time is from a steady clock
  // and relative to the publisher which is publishing it. If a message is transmitted over the
  // network its pubtime will be changed after arrival when it is published again in the local
  // system.
  int64_t pubtime;

  // A list of buffer objects associated with this message. Buffer objects can be used to store
  // various large memory objects like images. Serialized data can refer to buffers via an
  // index.
  std::vector<SharedBuffer> buffers;

  // An identifier which describes the type of the message. It is up to the specific sub classes
  // to make use of this identifier.
  uint64_t type;
};

using MessageBasePtr = std::shared_ptr<MessageBase>;
using ConstMessageBasePtr = std::shared_ptr<const MessageBase>;

// Creates a (deep) copy of a message. Not all message types might be supported and in that case
// this function will return nullptr.
MessageBasePtr Clone(ConstMessageBasePtr message);

// Message class with typed message contents. These messages are currently only supported for
// in-memory message passing.
template <typename T>
class RawMessage : public MessageBase {
 public:
  RawMessage() {}
  RawMessage(const T& data) : data(data) {}
  RawMessage(T&& data) : data(std::forward<T>(data)) {}

  size_t getPayloadSize() const override {
    // FIXME This might not be accurate
    return sizeof(T);
  }

  T data;
};

// A raw message containing a Json object
using JsonMessage = RawMessage<nlohmann::json>;

// Message for sight operations
// TODO This should be replaced with JSON message
struct SightMessage : public MessageBase {
  size_t getPayloadSize() const override {
    // FIXME Can not be implemented easily without expensive computations
    return json.size() * 10;
  }

  Json json;
};

// Calls getRoot() immediately after constructing MessageReader to work-around thread-safety issue:
// First call to getRoot() constructs capnp::ReaderArena. For more details, please see
// https://groups.google.com/forum/#!topic/capnproto/TpzRe42Hd48
struct CapnpReader {
  template <typename... Args>
  CapnpReader(Args&&... args) : reader(std::forward<Args>(args)...) {
    reader.getRoot<capnp::AnyPointer>();
  }

  ::capnp::SegmentArrayMessageReader reader;
};

// Base class for cap'n'proto messages
class ProtoMessageBase : public MessageBase {
 public:
  virtual ~ProtoMessageBase() = default;

  // Unique identifier from the proto schema file
  uint64_t proto_id() const { return type; }
  // A cap'n'proto reader for reading the message contents
  ::capnp::SegmentArrayMessageReader& reader() const {
    ASSERT(capnp_reader_ != nullptr, "Reader not initialized");
    return capnp_reader_->reader;
  }

  // The cap'n'proto segments of this message
  virtual kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments() const = 0;

  size_t getPayloadSize() const override;

 protected:
  std::unique_ptr<CapnpReader> capnp_reader_;
};

using ProtoMessageBasePtr = std::shared_ptr<ProtoMessageBase>;
using ConstProtoMessageBasePtr = std::shared_ptr<const ProtoMessageBase>;

// A proto message which holds a link to another proto message
class DependentProtoMessageBase : public ProtoMessageBase {
 public:
  DependentProtoMessageBase(ConstProtoMessageBasePtr child);

  kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments() const override {
    return child_->segments();
  }

 private:
  ConstProtoMessageBasePtr child_;
};

// A proto message which uses a malloc message builder to store data. Mostly used when building
// messages from scratch.
class MallocProtoMessage : public ProtoMessageBase {
 public:
  // Message is constructed with a new malloc message builder
  MallocProtoMessage(uint64_t proto_id);
  // Message is constructed based on an existing malloc message builder
  MallocProtoMessage(std::unique_ptr<::capnp::MallocMessageBuilder> malloc_message_builder,
                     uint64_t proto_id);

  kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments() const override;

 private:
  std::unique_ptr<::capnp::MallocMessageBuilder> malloc_message_builder_;
};

// A proto message which uses a single continuous buffer as data storage. Used when parsing
// a message from a socket, or other memory.
class BufferedProtoMessage : public ProtoMessageBase {
 public:
  // Message is constructed based on packages received from a UDP socket
  BufferedProtoMessage(std::vector<uint8_t> buffer, size_t offset,
                       std::vector<size_t> segment_lengths);

  // Ctor from segments
  BufferedProtoMessage(std::vector<CpuBuffer>&& segments);

  // A list of pointer pairs to the segments of the message
  kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments() const override;

  // A buffer containing all data of the proto message
  const std::vector<uint8_t>& buffer() const { return buffer_; }
  std::vector<uint8_t>& buffer() { return buffer_; }

  void setProtoId(const uint64_t proto_id) { type = proto_id; }

 private:
  std::vector<uint8_t> buffer_;
  std::vector<kj::ArrayPtr<const ::capnp::word>> segments_;
};

}  // namespace alice
}  // namespace isaac
