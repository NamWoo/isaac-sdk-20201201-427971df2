/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Subprocess.hpp"

#include <cstdlib>

namespace isaac {
namespace alice {

void Subprocess::start() {
  const auto cmd = get_start_command();
  if (!cmd.empty()) {
    auto result = std::system(cmd.c_str());
    if (result != 0) {
      LOG_ERROR("Command %s Failed with code 0x%x", cmd.c_str(), result);
    }
  }
}

void Subprocess::stop() {
  const auto cmd = get_stop_command();
  if (!cmd.empty()) {
    auto result = std::system(cmd.c_str());
    if (result != 0) {
      LOG_ERROR("Command %s Failed with code 0x%x", cmd.c_str(), result);
    }
  }
}

}  // namespace alice
}  // namespace isaac
