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
#include <utility>
#include <vector>

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/sight/sight_interface.hpp"
#include "engine/gems/sight/sop.hpp"

namespace isaac {
namespace sight {

// Reset sight handler. This call gives up the ownership of sight
void ResetSight(SightInterface* sight);

// Plots timeseries at a given time.
void Plot(const std::string& name, double time, float value);
void Plot(const std::string& name, double time, double value);
void Plot(const std::string& name, double time, int value);
void Plot(const std::string& name, double time, int64_t value);
inline void Plot(const std::string& name, double time, size_t value) {
  Plot(name, time, static_cast<int64_t>(value));
}

// Plots timeseries.
void Plot(const std::string& name, float value);
void Plot(const std::string& name, double value);
void Plot(const std::string& name, int value);
void Plot(const std::string& name, int64_t value);
void Plot(const std::string& name, size_t value);
inline void Plot(const std::string& name, size_t value) {
  Plot(name, static_cast<int64_t>(value));
}

// Draws on a canvas a list of operation.
void Draw(const std::string& name, sight::Sop sop);

// Displays an image with sight
void Draw(const std::string& name, const ImageConstView3ub& img);
void Draw(const std::string& name, const ImageConstView4ub& img);
void Draw(const std::string& name, const ImageConstView1ub& img);

}  // namespace sight
}  // namespace isaac
