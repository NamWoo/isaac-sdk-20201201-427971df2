/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/pyalice/bindings/pybind_component.hpp"

#include "engine/alice/component.hpp"
#include "packages/pyalice/bindings/pybind_node.hpp"

namespace isaac {
namespace alice {

PybindComponent::PybindComponent() : component_(nullptr) {}

PybindComponent::PybindComponent(alice::Component* component) : component_(component) {}

PybindComponent::~PybindComponent() {}

const std::string& PybindComponent::name() const {
  return component_->name();
}

void InitPybindComponent(pybind11::module& m) {
  pybind11::class_<PybindComponent>(m, "PybindComponent")
      .def(pybind11::init<>())
      .def("name", &isaac::alice::PybindComponent::name);
}

}  // namespace alice
}  // namespace isaac
