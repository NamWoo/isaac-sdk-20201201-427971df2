/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <array>
#include <functional>

// A cache for 256 values which will give one value after another in round robin fashion.
template <typename K>
class Cache256 {
 public:
  // Create a new cache filling by repeatedly evaluating the given functor
  Cache256(std::function<K()> f) {
    std::generate(cache_.begin(), cache_.end(), f);
    index_ = 0;
  }
  // Get the next value
  K operator()() {
    return cache_[index_++];
  }

 private:
  std::array<K, 256> cache_;
  uint8_t index_;
};
