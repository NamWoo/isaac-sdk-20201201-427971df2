/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "pybind11/pybind11.h"

namespace isaac {
namespace alice {

class Component;
class PybindNode;
class PybindPyCodelet;

// A wrapper around an alice component to make it accessible to Python via pybind
class PybindComponent {
 public:
  PybindComponent();
  PybindComponent(alice::Component* component);
  ~PybindComponent();

  // Returns a pointer to the underlying component
  alice::Component* component() const { return component_; }

  // Calls Component::name
  const std::string& name() const;

 private:
  friend class PybindPyCodelet;
  alice::Component* component_;
};

// Initializes the python module
void InitPybindComponent(pybind11::module& m);

}  // namespace alice
}  // namespace isaac
