/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine/alice/backend/clock.hpp"
#include "packages/pyalice/bindings/pybind_application.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

// Pybind API to provide access to a cask log file in Python
class PybindClock {
 public:
  // Loads the cask at the given location
  PybindClock(const PybindApplication& app);
  ~PybindClock();

  // Returns the current app time in seconds. In case of null clock, returns negative number.
  double getTime();

 private:
  const Clock* clock_;
};

// Initializes the python module
void InitPybindClock(pybind11::module& module);

}  // namespace alice
}  // namespace isaac
