/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <algorithm>
#include <numeric>
#include <string>
#include <vector>

#include "benchmark/benchmark.h"
#include "capnp/serialize.h"
#include "engine/core/logger.hpp"
#include "engine/gems/serialization/header.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "messages/uuid.hpp"
#include "packages/engine_gems/benchmarks/serialization/Header.capnp.h"

void DummyData(isaac::serialization::Header& header) {
  header.timestamp = 123456789;
  header.uuid = isaac::Uuid::Generate();
  header.tag = "so long and thanks for all the fish";
  header.acqtime = 23486765552;
  header.format = 1337;
  header.minipayload.resize(12);
  std::iota(header.minipayload.begin(), header.minipayload.end(), 0);
  header.segments = {15664, 123, 17, 933};
  header.buffers = {3422, 1288493, 229434};
}

void DummyDataXL(isaac::serialization::Header& header) {
  header.timestamp = 123456789;
  header.uuid = isaac::Uuid::Generate();
  header.tag = "so long and thanks for all the fish";
  header.acqtime = 23486765552;
  header.format = 1337;
  header.minipayload.resize(135);
  std::iota(header.minipayload.begin(), header.minipayload.end(), 0);
  header.segments.resize(137);
  std::iota(header.segments.begin(), header.segments.end(), 0);
  header.buffers.resize(133);
  std::iota(header.buffers.begin(), header.buffers.end(), 0);
}

void Serialization_Header_Write(benchmark::State& state) {
  isaac::serialization::Header expected;
  DummyData(expected);
  for (auto _ : state) {
    std::vector<uint8_t> buffer;
    Serialize(expected, buffer);
  }
}

void Serialization_Header_Write_Reuse(benchmark::State& state) {
  isaac::serialization::Header expected;
  DummyData(expected);
  std::vector<uint8_t> buffer;
  for (auto _ : state) {
    Serialize(expected, buffer);
  }
}

void Serialization_Header_Read(benchmark::State& state) {
  isaac::serialization::Header expected;
  DummyData(expected);
  std::vector<uint8_t> buffer;
  Serialize(expected, buffer);
  for (auto _ : state) {
    isaac::serialization::Header actual;
    Deserialize(buffer, actual);
  }
}

void Serialization_Header_Read_Reuse(benchmark::State& state) {
  isaac::serialization::Header expected;
  DummyData(expected);
  std::vector<uint8_t> buffer;
  Serialize(expected, buffer);
  isaac::serialization::Header actual;
  for (auto _ : state) {
    Deserialize(buffer, actual);
  }
}

void Serialization_Header_Write_Reuse_XL(benchmark::State& state) {
  isaac::serialization::Header expected;
  DummyDataXL(expected);
  std::vector<uint8_t> buffer;
  for (auto _ : state) {
    Serialize(expected, buffer);
  }
}

void Serialization_Header_Read_Reuse_XL(benchmark::State& state) {
  isaac::serialization::Header expected;
  DummyDataXL(expected);
  std::vector<uint8_t> buffer;
  Serialize(expected, buffer);
  isaac::serialization::Header actual;
  for (auto _ : state) {
    Deserialize(buffer, actual);
  }
}

void Serialization_Header_Proto_Write(benchmark::State& state) {
  isaac::serialization::Header header;
  DummyData(header);
  for (auto _ : state) {
    ::capnp::MallocMessageBuilder builder;
    auto proto = builder.initRoot<HeaderProto>();
    proto.setTimestamp(*header.timestamp);
    isaac::alice::ToProto(*header.uuid, proto.initUuid());
    proto.setTag(header.tag);
    proto.setAcqtime(*header.acqtime);
    proto.initMinipayload(header.minipayload.size());
    auto minipayload = proto.getMinipayload();
    std::copy(header.minipayload.begin(), header.minipayload.end(), minipayload.begin());
    proto.initSegments(header.segments.size());
    auto segments = proto.getSegments();
    for (size_t i = 0; i < segments.size(); i++) {
      segments.set(i, header.segments[i]);
    }
    proto.initBuffers(header.buffers.size());
    auto buffers = proto.getBuffers();
    for (size_t i = 0; i < buffers.size(); i++) {
      buffers.set(i, header.buffers[i]);
    }
  }
}

BENCHMARK(Serialization_Header_Write);
BENCHMARK(Serialization_Header_Write_Reuse);
BENCHMARK(Serialization_Header_Write_Reuse_XL);
BENCHMARK(Serialization_Header_Read);
BENCHMARK(Serialization_Header_Read_Reuse);
BENCHMARK(Serialization_Header_Read_Reuse_XL);
BENCHMARK(Serialization_Header_Proto_Write);
BENCHMARK_MAIN();
