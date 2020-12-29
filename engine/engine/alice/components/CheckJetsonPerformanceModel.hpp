/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace alice {

// Checks whether the Jetson device is in the desired nvpmodel mode. Skips the check for non-Jetson
// devices. Reads the file with nvpmodel mode and reports failure or interrupts the program if the
// mode does not equal to the parameterized value.
class CheckJetsonPerformanceModel : public Codelet {
 public:
  void start() override;
  void tick() override;

  // Path to the file that shows the current nvpmodel mode
  ISAAC_PARAM(std::string, status_filename, "/var/lib/nvpmodel/status");
  // This codelet will check whether the nvpmodel mode equals this value.
  // 0 means maximum power and performance.
  ISAAC_PARAM(int, desired_status, 0);
  // Sets whether to assert or report failure when the nvpmodel mode does not match desired_status.
  ISAAC_PARAM(bool, assert_on_undesired_status, true);
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::CheckJetsonPerformanceModel)
