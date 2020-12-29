/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/components/Codelet.hpp"

namespace isaac {
namespace alice {

// Periodically writes statistic about allocated buffers to the redis metadata server.
// The buffer allocator report will be stored in a hashset under the following keys:
//   isaac:AUUID:mem
//     count: total number of allocation requests over app lifetime
//     rate: number of allocations per second over app lifetime
//     bytes_requested: total number of bytes requested over app lifetime
//     bytes_allocated: total number of bytes allocated over app lifetime
//     bytes_deallocated: total number of bytes deallocated over app lifetime
//     duration: total uptime of allocator manager
//     efficiency: percentage of requests which were served without performing an allocation
class BufferAllocatorReport : public Codelet {
 public:
  void start() override;
  void tick() override;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::BufferAllocatorReport)
