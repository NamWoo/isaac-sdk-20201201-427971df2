/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

namespace isaac {
namespace scheduler {

// This descriptor describes the execution groups for the scheduler.
// Each named group will generate an execution unit that controls the
// lifetime of the jobs and threads associated with it.
struct ExecutionGroupDescriptor {
  // The name of the execution group. Used to index groups against jobs.
  std::string name;
  // The cores that the group will execute on.
  std::vector<int> cores;
  // Whether a group will create worker threads or not.
  bool has_workers;
};

}  // namespace scheduler
}  // namespace isaac
