/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/pyalice/bindings/pybind_clock.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/clock.hpp"
#include "engine/alice/component_impl.hpp"
#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

PybindClock::PybindClock(const PybindApplication& app) {
  clock_ = app.app().backend()->clock();
  ASSERT(clock_ != nullptr, "Invalid Clock pointer");
}

PybindClock::~PybindClock() {}

double PybindClock::getTime() {
  return clock_->time();
}

void InitPybindClock(pybind11::module& m) {
  pybind11::class_<PybindClock>(m, "PybindClock")
      .def(pybind11::init<const PybindApplication&>())
      .def("get_time", &PybindClock::getTime);
}

}  // namespace alice
}  // namespace isaac
