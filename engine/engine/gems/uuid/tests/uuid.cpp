/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <mutex>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

#include "engine/core/logger.hpp"
#include "engine/gems/uuid/uuid.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Uuid, Basics) {
 Uuid a = Uuid::Generate();
 EXPECT_EQ(a, a);
 Uuid b = a;
 EXPECT_EQ(a, b);
 EXPECT_TRUE(a == b);
 EXPECT_FALSE(a != b);
 Uuid c = Uuid::Generate();
 EXPECT_NE(a, c);
 EXPECT_TRUE(a != c);
 EXPECT_FALSE(a == c);
}

TEST(Uuid, Print) {
  Uuid a = Uuid::Generate();
  LOG_INFO("UUID: `%s`", a.c_str());
}

TEST(Uuid, Set) {
  std::set<Uuid> set;
  for (int i=1; i<100; i++) {
    auto a = Uuid::Generate();
    set.insert(a);
    EXPECT_EQ(set.size(), i);
    set.insert(a);
    EXPECT_EQ(set.size(), i);
  }
}

TEST(Uuid, Uniqueness1) {
  constexpr int kCount = 1000;
  std::vector<Uuid> uuids;
  uuids.reserve(kCount);
  for (int i=0; i<kCount; i++) {
    uuids.push_back(Uuid::Generate());
  }
  std::set<Uuid> set(uuids.begin(), uuids.end());
  EXPECT_EQ(set.size(), kCount);
}

TEST(Uuid, Uniqueness2) {
  constexpr int kCount = 1000;
  std::set<Uuid> uuids;
  for (int i=0; i<kCount; i++) {
    uuids.insert(Uuid::Generate());
  }
  EXPECT_EQ(uuids.size(), kCount);
}

TEST(Uuid, UnorderedSet) {
  std::unordered_set<Uuid> set;
  for (int i=1; i<100; i++) {
    auto a = Uuid::Generate();
    set.insert(a);
    EXPECT_EQ(set.size(), i);
    set.insert(a);
    EXPECT_EQ(set.size(), i);
  }
}

TEST(Uuid, Uniqueness3) {
  constexpr int kCount = 1000;
  std::unordered_set<Uuid> uuids;
  for (int i=0; i<kCount; i++) {
    uuids.insert(Uuid::Generate());
  }
  EXPECT_EQ(uuids.size(), kCount);
}

TEST(Uuid, OneMillion) {
  constexpr int kCount = 1e6;
  Uuid last;
  for (int i=0; i<kCount; i++) {
    auto x = Uuid::Generate();
    EXPECT_NE(last, x);
    last = x;
  }
}

TEST(Uuid, MultiThreadedGenerateUuid) {
  std::mutex mutex;
  std::unordered_set<Uuid> result;
  constexpr int kCount = 10000;
  constexpr int kNumThreads = 8;
  std::vector<std::thread> threads;
  for (int i=0; i<kNumThreads; i++) {
    threads.emplace_back([&] {
      std::set<Uuid> uuids;
      for (int i=0; i<kCount; i++) {
        uuids.insert(Uuid::Generate());
      }
      std::lock_guard<std::mutex> lock(mutex);
      result.insert(uuids.begin(), uuids.end());
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  threads.clear();
  EXPECT_EQ(result.size(), kNumThreads * kCount);
}

TEST(Uuid, Str) {
  Uuid a = Uuid::Generate();
  EXPECT_EQ(a.str(), std::string(a.c_str()));
  EXPECT_EQ(a.str().c_str(), a.c_str());
}

}  // namespace isaac
