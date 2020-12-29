/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/component_impl.hpp"

namespace isaac {
namespace alice {

// Base class for a hook, for example for message, configuration or pose hooks.
class Hook {
  friend class Component;  // for calling connect
  friend class PyCodelet;  // for calling connect

 public:
  Hook(Component* component);
  ~Hook();

  // The component to which this hook is connected
  Component* component() const { return component_; }

 protected:
  // Connects the hook to its component
  virtual void connect() {}

 private:
  Component* component_;
};

}  // namespace alice
}  // namespace isaac
