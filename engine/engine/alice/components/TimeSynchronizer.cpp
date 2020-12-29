/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "TimeSynchronizer.hpp"

#include "engine/alice/node.hpp"
#include "engine/core/time.hpp"

namespace isaac {
namespace alice {

void TimeSynchronizer::start() {
  offset_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count() -
                 node()->clock()->timestamp();
  reportSuccess();
}

int64_t TimeSynchronizer::appToSyncTime(int64_t app_time) const {
  return app_time + offset_time_;
}

int64_t TimeSynchronizer::syncToAppTime(int64_t sync_time) const {
  return sync_time - offset_time_;
}

}  // namespace alice
}  // namespace isaac
