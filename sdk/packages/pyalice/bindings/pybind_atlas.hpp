/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <memory>
#include <string>

#include "engine/alice/backend/clock.hpp"
#include "engine/alice/components/PoseTree.hpp"
#include "packages/pyalice/bindings/pybind_application.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

// Pybind API to provide access to a cask log file in Python
class PybindAtlas {
 public:
  // Loads the cask at the given location
  PybindAtlas(const PybindApplication& app);
  ~PybindAtlas();

  // Gets pose lhs_T_rhs for given time in seconds. Output is [qw, qx, qy, qz, x, y, z]
  pybind11::object getPose(const std::string& lhs, const std::string& rhs, const double time);

  // Sets pose lhs_Trhs for given time in seconds. Pose is [qw, qx, qy, qz, x, y, z]
  bool setPose(const std::string& lhs, const std::string& rhs, const double time,
               const std::array<double, 7>& pose);

 private:
  PoseTree* pose_tree_;
};

// Initializes the python module
void InitPybindAtlas(pybind11::module& module);

}  // namespace alice
}  // namespace isaac
