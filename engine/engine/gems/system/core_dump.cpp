/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "core_dump.hpp"

#include <sys/resource.h>

#include "engine/core/logger.hpp"

void EnableCoreDump() {
  struct rlimit core_limit;
  core_limit.rlim_cur = RLIM_INFINITY;
  core_limit.rlim_max = RLIM_INFINITY;

  if (setrlimit(RLIMIT_CORE, &core_limit) < 0) {
    LOG_ERROR("Unable to enable core dumps fully. Dumps may be limited or empty.");
  }
}
