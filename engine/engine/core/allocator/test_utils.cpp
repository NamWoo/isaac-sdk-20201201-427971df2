/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "test_utils.hpp"

#include <algorithm>
#include <random>
#include <utility>
#include <vector>

namespace isaac {

void BatchAllocDealloc(AllocatorBase& allocator, int count, int batch,
                       std::function<size_t()> size_cb) {
  std::vector<std::pair<float*, size_t>> ptrs;
  ptrs.reserve(batch);
  for (int i = 0; i < count; i++) {
    ptrs.clear();
    for (int j = 0; j < batch; j++) {
      const size_t size = size_cb();
      if (size == 0) continue;
      ptrs.push_back({allocator.allocate<float>(size), size});
    }
    for (const auto& kvp : ptrs) {
      allocator.deallocate<float>(kvp.first, kvp.second);
    }
  }
}

std::function<size_t()> RandomSizeGamma(size_t pool_size) {
  constexpr size_t kRounder = 1024;
  constexpr size_t kFactor = 32;
  if (pool_size == 0) {
    return [rng = std::default_random_engine{},
            random = std::gamma_distribution<double>{2.0, 2.0}] () mutable {
      const size_t q = static_cast<size_t>(random(rng) * kRounder) / kRounder;
      return (q + 1) * kRounder * kFactor;
    };
  } else {
    std::default_random_engine rng;
    std::gamma_distribution<double> random{2.0, 2.0};
    std::vector<size_t> pool(pool_size);
    std::generate(pool.begin(), pool.end(), [&] {
      const size_t q = static_cast<size_t>(random(rng) * kRounder) / kRounder;
      return (q + 1) * kRounder * kFactor;
    });
    return [pool = std::move(pool), i = 0] () mutable {
      return pool[(i++) % pool.size()];
    };
  }
}

}  // namespace isaac
