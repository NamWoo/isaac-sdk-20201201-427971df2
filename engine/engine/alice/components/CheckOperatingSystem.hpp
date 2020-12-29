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

// Checks whether the operating system meets the requirements.
// For the host, this means the right version of Ubuntu.
// For the Jetson devices, this means the right version of L4T or Jetpack.
// To ensure safety and performance, this component report failure
// if the operating system is not compatible or it cannot be verified.
class CheckOperatingSystem : public Codelet {
 public:
  void start() override;

  // File to read to check operating system of the host
  ISAAC_PARAM(std::string, host_release_file, "/etc/lsb-release");
  // Key of operating system id
  ISAAC_PARAM(std::string, host_id_field, "DISTRIB_ID");
  // Value of operating system id
  ISAAC_PARAM(std::string, host_id_value, "Ubuntu");
  // Key that shows the operating system version
  ISAAC_PARAM(std::string, host_release_field, "DISTRIB_RELEASE");
  // Value of operating system version
  ISAAC_PARAM(std::string, host_release_value, "18.04");

  // File to read to check operating system of a jetson
  ISAAC_PARAM(std::string, jetson_release_file, "/etc/nv_tegra_release");
  // Substring that is expected from the desired L4T release
  ISAAC_PARAM(std::string, jetson_release_version, "# R32 (release), REVISION: 4.4");
  // Desired jetpack version. Used only when printing error.
  ISAAC_PARAM(std::string, jetpack_version, "4.4.1");

  // CPU architecture of the host
  ISAAC_PARAM(std::string, host_architecture, "x86_64");
  // CPU architecture of Jetsons
  ISAAC_PARAM(std::string, jetson_architecture, "aarch64");

  // CUDA version (10020 = 10.2)
  ISAAC_PARAM(int, cuda_version, 10020);

 private:
  // Reports failure the program if the host does not have the expected operating system.
  void checkHost();
  // Reports failure the program if the Jetson device does not have the expected operating system.
  void checkJetson();
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::CheckOperatingSystem)
