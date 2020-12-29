/*
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/pyalice/bindings/pybind_application.hpp"
#include "packages/pyalice/bindings/pybind_atlas.hpp"
#include "packages/pyalice/bindings/pybind_clock.hpp"
#include "packages/pyalice/bindings/pybind_component.hpp"
#include "packages/pyalice/bindings/pybind_core.hpp"
#include "packages/pyalice/bindings/pybind_logger.hpp"
#include "packages/pyalice/bindings/pybind_message.hpp"
#include "packages/pyalice/bindings/pybind_node.hpp"
#include "packages/pyalice/bindings/pybind_py_codelet.hpp"
#include "pybind11/pybind11.h"

#include <string>
#include "engine/version.hpp"

PYBIND11_MODULE(bindings, m) {
  m.doc() = R"pbdoc(
        Isaac Alice Python Bridge
        -----------------------

        .. currentmodule:: pyalice

    )pbdoc";

  isaac::alice::InitPybindApplication(m);
  isaac::alice::InitPybindAtlas(m);
  isaac::alice::InitPybindClock(m);
  isaac::alice::InitPybindComponent(m);
  isaac::alice::InitPybindCore(m);
  isaac::alice::InitPybindLogger(m);
  isaac::alice::InitPybindMessage(m);
  isaac::alice::InitPybindNode(m);
  isaac::alice::InitPybindPyCodelet(m);

  // Version Query
  m.def("isaac_sdk_version", []() { return std::string(IsaacSdkVersion()); });
}
