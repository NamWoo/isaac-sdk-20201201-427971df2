/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "time.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <string>
#include <thread>

namespace isaac {

int64_t NowCount() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::string GetCurrentDateTime() {
  std::time_t t = std::time(nullptr);
  std::tm* now = std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(now, "%F %T");
  return oss.str();
}

void Sleep(int64_t dt_nano) {
  std::this_thread::sleep_for(std::chrono::duration<int64_t, std::nano>(dt_nano));
}

std::optional<double> ParseDurationStringToSecond(const std::string& duration_string) {
  std::optional<double> result = std::nullopt;
  if (duration_string.length() > 1) {
    const char unit = duration_string.back();
    switch (unit) {
      case 's':
        result = 1.0f;
        break;
      case 'm':
        result = 60.0f;
        break;
      case 'h':
        result = 60.0f * 60.0f;
        break;
      default:
        result = std::nullopt;
        break;
    }
    if (result) {
      result = (*result) * std::atof(duration_string.c_str());
      if (std::isnan(*result)) {
        result = std::nullopt;
      }
    }
  }
  return result;
}

}  // namespace isaac
