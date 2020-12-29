/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CheckOperatingSystem.hpp"

#include <sys/utsname.h>

#include <fstream>
#include <string>
#include <vector>

#include "cuda.h"
#include "cuda_runtime_api.h"

namespace isaac {
namespace alice {

void CheckOperatingSystem::start() {
  // Check depending on the architecture
  struct utsname unameData;
  const int uname_result = uname(&unameData);
  if (uname_result != 0) {
    reportFailure("Failed to get architecture information.");
    return;
  }
  const std::string architecture = unameData.machine;
  if (architecture == get_host_architecture()) {
    checkHost();
  } else if (architecture == get_jetson_architecture()) {
    checkJetson();
  } else {
    reportFailure("Unknown architecture '%s'.", unameData.machine);
    return;
  }
  // Checks CUDA version
  int cuda_runtime_version = 0;
  const auto result = ::cudaRuntimeGetVersion(&cuda_runtime_version);
  if (result != cudaSuccess) {
    reportFailure("Failed to retrieve CUDA runtime version: %d", result);
    return;
  }
  if (cuda_runtime_version != CUDA_VERSION) {
    reportFailure("Mismatching CUDA runtime version: %d not matching built with: %d",
                  cuda_runtime_version, CUDA_VERSION);
    return;
  }
  if (cuda_runtime_version != get_cuda_version()) {
    reportFailure("Mismatching CUDA runtime version: %d not matching expected: %d",
                  cuda_runtime_version, get_cuda_version());
    return;
  }
}

void CheckOperatingSystem::checkHost() {
  // Read parameters
  const std::string host_release_file = get_host_release_file();
  const std::string host_id_field = get_host_id_field();
  const std::string host_release_field = get_host_release_field();
  const std::string host_id_value = get_host_id_value();
  const std::string host_release_value = get_host_release_value();

  // Open file for reading
  std::ifstream file(host_release_file);
  if (!file) {
    reportFailure("Failed to open '%s' for reading.", host_release_file.c_str());
    return;
  }

  // Read required fields from file
  std::string id;
  std::string release;
  std::string line;
  while (std::getline(file, line)) {
    const std::vector<std::string> tokens = SplitString(line, '=');
    if (tokens.size() == 2) {
      if (tokens[0] == host_id_field) {
        if (!id.empty()) {
          reportFailure("More than one '%s' fields in '%s'.", host_id_field.c_str(),
                        host_release_file.c_str());
          return;
        }
        id = tokens[1];
      } else if (tokens[0] == host_release_field) {
        if (!release.empty()) {
          reportFailure("More than one '%s' fields in '%s'.", host_release_field.c_str(),
                        host_release_file.c_str());
          return;
        }
        release = tokens[1];
      }
    }
  }

  // Check operating system id and version
  if (id.empty()) {
    reportFailure("No '%s' field in '%s'.", host_id_field.c_str(), host_release_file.c_str());
    return;
  }
  if (release.empty()) {
    reportFailure("No '%s' field in '%s'.", host_release_field.c_str(), host_release_file.c_str());
    return;
  }
  if (id != host_id_value || release != host_release_value) {
    reportFailure("Isaac requires %s %s while the host has %s %s.", host_id_value.c_str(),
                  host_release_value.c_str(), id.c_str(), release.c_str());
    return;
  }
}

void CheckOperatingSystem::checkJetson() {
  // Read parameters
  const std::string jetson_release_file = get_jetson_release_file();
  const std::string jetson_release_version = get_jetson_release_version();
  const std::string jetpack_version = get_jetpack_version();

  // Open file for reading
  std::ifstream file(jetson_release_file);
  if (!file) {
    reportFailure("Failed to open '%s' for reading.", jetson_release_file.c_str());
    return;
  }

  // Read first line from file
  std::string line;
  std::getline(file, line);

  // Check operating system string
  if (line.empty()) {
    reportFailure("First line of '%s' is empty.", jetson_release_file.c_str());
    return;
  }
  if (line.find(jetson_release_version) != 0) {
    reportFailure("'%s' does not start with '%s'. Please flash L4T using Jetpack %s.",
                  jetson_release_file.c_str(), jetson_release_version.c_str(),
                  jetpack_version.c_str());
    return;
  }
}

}  // namespace alice
}  // namespace isaac
