/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "message.hpp"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

size_t MessageBase::getBufferSize() const {
  size_t size = 0;
  for (const auto& buffer : buffers) {
    size += buffer.size();
  }
  return size;
}

size_t MessageBase::size() const {
  return getBufferSize() + getPayloadSize() + sizeof(MessageBase);
}

MessageBasePtr Clone(ConstMessageBasePtr message) {
  MessageBasePtr result;
  if (auto proto_message = std::dynamic_pointer_cast<const ProtoMessageBase>(message)) {
    result = std::make_shared<DependentProtoMessageBase>(proto_message);
  }
  if (!result) {
    return nullptr;
  }
  // copy base class
  result->uuid = Uuid::Generate();
  result->acqtime = message->acqtime;
  result->pubtime = message->pubtime;
  // Copy buffers
  // FIXME Do not copy buffers and augement message with an interface using views instead.
  for (size_t i = 0; i < result->buffers.size(); i++) {
    const auto& rhs = message->buffers[i].host_buffer();
    CpuBuffer lhs(rhs.size());
    std::copy(rhs.begin(), rhs.end(), lhs.begin());
    result->buffers.emplace_back(std::move(lhs));
  }
  return result;
}

size_t ProtoMessageBase::getPayloadSize() const {
  size_t size = 0;
  for (const auto& segment : segments()) {
    size += segment.size() * sizeof(::capnp::word);
  }
  return size;
}

DependentProtoMessageBase::DependentProtoMessageBase(ConstProtoMessageBasePtr child)
    : child_(std::move(child)) {
  type = child_->proto_id();
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  capnp_reader_ = std::make_unique<CapnpReader>(child_->segments(), options);
}

MallocProtoMessage::MallocProtoMessage(uint64_t proto_id)
    : MallocProtoMessage(std::make_unique<::capnp::MallocMessageBuilder>(), proto_id) {
  type = proto_id;
}

MallocProtoMessage::MallocProtoMessage(
    std::unique_ptr<::capnp::MallocMessageBuilder> malloc_message_builder, uint64_t proto_id)
    : malloc_message_builder_(std::move(malloc_message_builder)) {
  type = proto_id;
  ASSERT(malloc_message_builder_, "Must not be nullptr");
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  capnp_reader_ =
      std::make_unique<CapnpReader>(malloc_message_builder_->getSegmentsForOutput(), options);
}

kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> MallocProtoMessage::segments() const {
  return malloc_message_builder_->getSegmentsForOutput();
}

BufferedProtoMessage::BufferedProtoMessage(std::vector<uint8_t> buffer, size_t offset,
                                           std::vector<size_t> segment_lengths) {
  type = 0;
  buffer_ = std::move(buffer);
  // Prepare pointer to the segments (there is a header at the start of each package)
  segments_.reserve(segment_lengths.size());
  for (size_t i = 0; i < segment_lengths.size(); i++) {
    const size_t length = segment_lengths[i];
    const size_t seg_begin = offset;
    const size_t seg_end = offset + length;
    offset += length;
    ASSERT(seg_begin % sizeof(::capnp::word) == 0, "corrupted data");
    ASSERT(seg_end % sizeof(::capnp::word) == 0, "corrupted data");
    const ::capnp::word* buffer_words_begin =
        reinterpret_cast<const ::capnp::word*>(buffer_.data() + seg_begin);
    const ::capnp::word* buffer_words_end =
        reinterpret_cast<const ::capnp::word*>(buffer_.data() + seg_end);
    segments_.push_back(kj::ArrayPtr<const ::capnp::word>(buffer_words_begin, buffer_words_end));
  }
  // Create a message reader from the segments
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  capnp_reader_.reset(new CapnpReader(segments(), options));
}

BufferedProtoMessage::BufferedProtoMessage(std::vector<CpuBuffer>&& segments_buffer) {
  type = 0;
  int64_t total_bytes_size = 0;
  for (const auto& segment_buffer : segments_buffer) {
    total_bytes_size += segment_buffer.size();
  }
  buffer_.resize(total_bytes_size);

  segments_.reserve(segments_buffer.size());
  int64_t offset = 0;
  for (const auto& segment_buffer : segments_buffer) {
    buffer_.assign(segment_buffer.pointer().get(),
                   segment_buffer.pointer().get() + segment_buffer.size());
    const ::capnp::word* segment_begin =
        reinterpret_cast<const ::capnp::word*>(buffer_.data() + offset);
    offset += segment_buffer.size();
    const ::capnp::word* segment_end =
        reinterpret_cast<const ::capnp::word*>(buffer_.data() + offset);

    segments_.push_back(kj::ArrayPtr<const ::capnp::word>(segment_begin, segment_end));
  }
  // Create a message reader from the segments
  ::capnp::ReaderOptions options;
  options.traversalLimitInWords = kj::maxValue;
  capnp_reader_.reset(new CapnpReader(segments(), options));
}

kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> BufferedProtoMessage::segments() const {
  return kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>>(segments_.data(), segments_.size());
}

}  // namespace alice
}  // namespace isaac
