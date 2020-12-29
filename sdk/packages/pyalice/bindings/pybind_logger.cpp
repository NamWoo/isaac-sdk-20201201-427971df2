/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_logger.hpp"

#include <vector>

#include "pybind11/pybind11.h"

#include "engine/core/logger.hpp"

namespace isaac {
namespace alice {

void InitPybindLogger(pybind11::module& m) {
  // Exports Enum Severity
  pybind11::enum_<logger::Severity>(m, "severity")
      .value("NONE", logger::Severity::NONE)
      .value("ALL", logger::Severity::ALL)
      .value("PANIC", logger::Severity::PANIC)
      .value("ERROR", logger::Severity::ERROR)
      .value("WARNING", logger::Severity::WARNING)
      .value("INFO", logger::Severity::INFO)
      .value("DEBUG", logger::Severity::DEBUG)
      .export_values();
  // Exports function SetSeverity()
  m.def("set_severity", &logger::SetSeverity);
}

}  // namespace alice
}  // namespace isaac
