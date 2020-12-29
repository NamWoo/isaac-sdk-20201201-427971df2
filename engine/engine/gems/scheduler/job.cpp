/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "job.hpp"

#include <memory>
#include <mutex>
#include <utility>

#include "engine/core/time.hpp"

namespace isaac {
namespace scheduler {

void Job::run() {
  // Mark job as executing
  running.store(true);
  // Double check the tombstone before running.
  if (!tombstone && scheduled) {
    // Execute job action
    description.action();
  }
  // Mark job as not executing
  running.store(false);
}

}  // namespace scheduler
}  // namespace isaac
