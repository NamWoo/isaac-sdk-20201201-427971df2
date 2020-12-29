/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/alice/backend/post_execution_report.hpp"

#include <cstdlib>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "engine/core/logger.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

namespace {

std::string StringifyCodeletStatistics(const nlohmann::json& json) {
  // Format the report and print on the console
  constexpr char kStatisticsHeader[] =
      "|=====================================================================================================================|\n"  // NOLINT
      "|                                           Job Statistics Report (regular)                                           |\n"  // NOLINT
      "|=====================================================================================================================|\n"  // NOLINT
      "| Name                                               |   Count | Time (Median - 90% - Max) [ms] | Load (%) | Late (%) |\n"  // NOLINT
      "|---------------------------------------------------------------------------------------------------------------------|\n";  // NOLINT

  constexpr char kStatisticsItemLine[] =
      "| %50.50s | %7zd | %8.2f | %8.2f | %8.2f | %6.1f %% | %6.1f %% |\n";  // NOLINT

  constexpr char kStatisticsFooter[] =
      "|=====================================================================================================================|";  // NOLINT

  const size_t kLineLength = sizeof(kStatisticsFooter);
  const size_t kSaveLineLength = 2 * kLineLength;  // give a lot of fudge for accidents

  // Compute total time spent in all codelets
  double total_time = 0;
  for (const auto& node_item : json.items()) {
    if (node_item.value().is_null()) continue;
    for (const auto& item : node_item.value().items()) {
      if (item.value().is_null()) continue;
      total_time += item.value()["total_time"].get<double>();
    }
  }

  std::vector<char> buffer;
  std::string report = kStatisticsHeader;

  for (const auto& node_item : json.items()) {
    if (node_item.value().is_null()) continue;
    for (const auto& item : node_item.value().items()) {
      if (item.value().is_null()) continue;
      buffer.resize(kSaveLineLength);
      const size_t used_size = std::snprintf(buffer.data(), buffer.size(), kStatisticsItemLine,
          TakeLast(item.key(), 50).c_str(),
          item.value()["num_ticks"].get<int64_t>(),
          item.value()["median"].get<double>(),
          item.value()["p90"].get<double>(),
          item.value()["max"].get<double>(),
          item.value()["total_time"].get<double>() / total_time * 100.0,
          item.value()["late_p"].get<double>());
      report += std::string(buffer.data(), used_size);
    }
  }
  report += kStatisticsFooter;

  return report;
}

}  // namespace

std::map<std::string, std::string> StringifyBackendStatistics(const nlohmann::json& json) {
  std::map<std::string, std::string> result;

  try {
    result["codelets"] = StringifyCodeletStatistics(json.at("codelets"));
  } catch (...) {
    LOG_ERROR("Could not stringify codelets statistics");
  }

  return result;
}

}  // namespace alice
}  // namespace isaac
