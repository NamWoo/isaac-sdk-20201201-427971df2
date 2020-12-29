/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <string>

#include "engine/core/optional.hpp"

namespace isaac {

// Returns timestamp in nanoseconds
int64_t NowCount();

// Converts a time duration from nano seconds to seconds
constexpr inline double ToSeconds(int64_t dt) {
  return static_cast<double>(dt) / 1.0e9;
}

// Converts a time duration from seconds to nanoseconds
constexpr inline int64_t SecondsToNano(double dt) {
  return static_cast<int64_t>(dt * 1.0e9 + 0.5);
}

// Sleeps for a time period (in nanoseconds)
void Sleep(int64_t dt_nano);

// Return current Date and Time in Y-M-D H:M:S format
std::string GetCurrentDateTime();

// Parse string of format "<float number>[smh]" to number of seconds
std::optional<double> ParseDurationStringToSecond(const std::string& duration);

}  // namespace isaac
