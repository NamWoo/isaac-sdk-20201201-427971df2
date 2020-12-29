/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <unordered_map>

#include "capnp/dynamic.h"

#include "engine/core/optional.hpp"
#include "engine/core/singleton.hpp"

namespace isaac {
namespace alice {

// Builder Entrance
std::optional<::capnp::DynamicStruct::Builder> GetRootBuilderByTypeId(
    const uint64_t type_id, ::capnp::MallocMessageBuilder& message_builder);

// Reader Entrance
std::optional<::capnp::DynamicStruct::Reader> GetRootReaderByTypeId(
    const uint64_t type_id, ::capnp::SegmentArrayMessageReader& message_builder);

// Central registry for getting root of Proto Builder/Reader
class ProtoRegistry {
 public:
  // Registers
  template <typename PROTO>
  int add() {
    builders_[::capnp::typeId<PROTO>()] = [](::capnp::MallocMessageBuilder& message_builder) {
      return message_builder.getRoot<PROTO>();
    };
    readers_[::capnp::typeId<PROTO>()] = [](::capnp::SegmentArrayMessageReader& message_reader) {
      return message_reader.getRoot<PROTO>();
    };
    return counter_++;
  }

  std::optional<::capnp::DynamicStruct::Builder> getRootBuilderByTypeId(
      const uint64_t type_id, ::capnp::MallocMessageBuilder& message_builder) {
    const auto it = builders_.find(type_id);
    if (it == builders_.end()) return std::nullopt;
    return it->second(message_builder);
  }
  std::optional<::capnp::DynamicStruct::Reader> getRootReaderByTypeId(
      const uint64_t type_id, ::capnp::SegmentArrayMessageReader& message_reader) {
    const auto it = readers_.find(type_id);
    if (it == readers_.end()) return std::nullopt;
    return it->second(message_reader);
  }

  using BuilderFunction = std::function<::capnp::DynamicStruct::Builder(
      ::capnp::MallocMessageBuilder& message_builder)>;
  using ReaderFunction = std::function<::capnp::DynamicStruct::Reader(
      ::capnp::SegmentArrayMessageReader& message_builder)>;

 private:
  std::unordered_map<uint64_t, BuilderFunction> builders_;
  std::unordered_map<uint64_t, ReaderFunction> readers_;
  int counter_ = 0;
};

namespace proto_registry_details {

template <typename PROTO>
struct ProtoRegistryInit {
  static int r;
};

template <typename PROTO>
int ProtoRegistryInit<PROTO>::r = Singleton<ProtoRegistry>::Get().add<PROTO>();

#define ISAAC_ALICE_REGISTER_PROTO(PROTO)  \
  namespace isaac {                        \
  namespace alice {                        \
  namespace proto_registry_details {       \
  template class ProtoRegistryInit<PROTO>; \
  }                                        \
  }                                        \
  }

}  // namespace proto_registry_details

}  // namespace alice
}  // namespace isaac
