/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/algorithm/flow_control.hpp"

#include <random>
#include <set>
#include <vector>

#include "gtest/gtest.h"

namespace isaac {

namespace {

std::mt19937 rng;

// We are using a set so that items get automatically sorted by the first element in the tuple which
// in our case is the timestamp. This is important when adding multiple streams after each other.
using DataStream = std::set<std::tuple<int64_t, int64_t, std::string>>;

void AddDataStream(const std::string& channel, double frequency, int64_t size, double duration,
               DataStream& data_streams) {
  std::normal_distribution<double> distribution(1.0 / frequency, 0.5 / frequency);
  for (double time = 0.0; time <= duration; time += distribution(rng)) {
    data_streams.insert(std::make_tuple(SecondsToNano(time), size, channel));
  }
}

}  // namespace

TEST(FlowControl, NonSaturated) {
  FlowControl<std::string> flow_control(10'000.0); // 10 kb/s
  DataStream data_streams;
  AddDataStream("C1", 10.0, 100, 10.0, data_streams); // 1kb/s
  AddDataStream("C2", 20.0, 100, 10.0, data_streams); // 2kb/s
  AddDataStream("C3", 30.0, 50, 10.0, data_streams); // 1.5kb/s
  AddDataStream("C4", 10.0, 200, 10.0, data_streams); // 2kb/s
  AddDataStream("C5", 10.0, 250, 10.0, data_streams); // 2.5kb/s
  // Total is: 9kb/s
  for (const auto& job : data_streams) {
    EXPECT_TRUE(flow_control.keepMessage(std::get<2>(job), std::get<0>(job), std::get<1>(job)));
  }
}

TEST(FlowControl, OneSaturated) {
  const double time = 100.0;
  const double target_bandwidth = 100'000.0;
  FlowControl<std::string> flow_control(target_bandwidth); // 100 kb/s
  DataStream data_streams;
  AddDataStream("C1", 100.0, 2000, time, data_streams); // 200kb/s
  int64_t bandwidth = 0;
  for (const auto& job : data_streams) {
    if (flow_control.keepMessage(std::get<2>(job), std::get<0>(job), std::get<1>(job))) {
      bandwidth += std::get<1>(job);
    }
  }
  EXPECT_LE(bandwidth, time * target_bandwidth * 1.05);
  EXPECT_GE(bandwidth * 1.05, time * target_bandwidth);
}

TEST(FlowControl, ManySaturated) {
  const double time = 100.0;
  const double target_bandwidth = 100'000.0;
  FlowControl<std::string> flow_control(target_bandwidth); // 100 kb/s
  DataStream data_streams;
  AddDataStream("C1", 100.0, 1000, time, data_streams); // 100kb/s
  AddDataStream("C2", 20.0, 1000, time, data_streams); // 20kb/s
  AddDataStream("C3", 30.0, 500, time, data_streams); // 15kb/s
  AddDataStream("C4", 10.0, 2000, time, data_streams); // 20kb/s
  AddDataStream("C5", 10.0, 2500, time, data_streams); // 25kb/s
  int64_t bandwidth = 0;
  for (const auto& job : data_streams) {
    if (flow_control.keepMessage(std::get<2>(job), std::get<0>(job), std::get<1>(job))) {
      bandwidth += std::get<1>(job);
    }
  }
  EXPECT_LE(bandwidth, time * target_bandwidth * 1.05);
  EXPECT_GE(bandwidth * 1.05, time * target_bandwidth);
}

TEST(FlowControl, BigMessages) {
  const double time = 100.0;
  const double target_bandwidth = 1'000'000.0;
  FlowControl<std::string> flow_control(target_bandwidth); // 1Mb/s
  DataStream data_streams;
  AddDataStream("C1", 10.0, 400000, time, data_streams); // 2Mb/s
  int64_t bandwidth = 0;
  for (const auto& job : data_streams) {
    if (flow_control.keepMessage(std::get<2>(job), std::get<0>(job), std::get<1>(job))) {
      bandwidth += std::get<1>(job);
    }
  }
  EXPECT_LE(bandwidth, time * target_bandwidth * 1.05);
  EXPECT_GE(bandwidth * 1.05, time * target_bandwidth);
}

}  // namespace isaac
