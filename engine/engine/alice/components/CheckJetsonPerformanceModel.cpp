/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "CheckJetsonPerformanceModel.hpp"

#include <sys/utsname.h>

#include <fstream>
#include <regex>
#include <string>

namespace isaac {
namespace alice {

void CheckJetsonPerformanceModel::start() {
  // Check architecture
  struct utsname unameData;
  const int uname_result = uname(&unameData);
  if (uname_result != 0) {
    reportFailure("Failed to get architecture information.");
    return;
  }
  const std::string architecture = unameData.machine;
  if (architecture == "x86_64") {
    // Nothing to check for the host machine.
    // nvpmodel mode is only for Jetson devices.
    reportSuccess();
    return;
  }
  if (architecture != "aarch64") {
    reportFailure("Unknown architecture '%s'.", unameData.machine);
    return;
  }

  tickPeriodically(1.0);
}

void CheckJetsonPerformanceModel::tick() {
  // Open file
  const std::string status_filename = get_status_filename();
  std::ifstream file(status_filename);
  if (!file) {
    reportFailure("Failed to open '%s' for reading.", status_filename.c_str());
    return;
  }

  // Read first line
  std::string first_line;
  const bool read_success = std::getline(file, first_line).eof();
  if (!read_success) {
    reportFailure("'%s' does not have a single line.", status_filename.c_str());
    return;
  }

  // Extract mode substring from first line.
  // Format for JetPack 4.3 is: "pmode:0000 fmode:quiet". More details can be found at:
  // https://tegra-sw-opengrok.nvidia.com/source/xref/stage-main/vendor/nvidia/tegra/power/nvpmodel/
  // We use the following regex:
  // First group is anything (for the beginning of the string)
  // Second group is "pmode:"
  // Third group is a sequence of numbers (describing the mode)
  // Fourth group is anything (for the remainder of the string)
  const std::regex re("(.*)(pmode:)([0-9]*) (.*)");
  const std::string status_string = std::regex_replace(first_line, re, "$3");

  // Read status
  int current_status = -1;
  bool convert_success = false;
  try {
    size_t pos;
    current_status = std::stoi(status_string, &pos);
    convert_success = pos == status_string.size();
  } catch (const std::exception&) {
    convert_success = false;
  }
  if (!convert_success) {
    reportFailure("Failed to extract mode from '%s'.", first_line.c_str());
    return;
  }

  // Check status
  const int desired_status = get_desired_status();
  if (current_status != desired_status) {
    reportFailure(
        "Current nvpmodel mode is %d. Please use the terminal command 'sudo nvpmodel -m %d' to "
        "switch to the tested and recommended mode.",
        current_status, desired_status);
    ASSERT(!get_assert_on_undesired_status(), "Undesired nvpmodel mode.");
  }
}

}  // namespace alice
}  // namespace isaac
