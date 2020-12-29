/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/alice/component.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

class Application;

// Base class for component backends
class ComponentBackendBase {
 public:
  virtual ~ComponentBackendBase() = default;
  // Name of the component type
  virtual const char* component_name() const = 0;
  // Called to start the backend (before any component is created)
  virtual void start() {}
  // Called to stop the backend (after all components are destroyed)
  virtual void stop() {}
  // Called to register a component with the backend
  virtual void registerComponent(Component* component) = 0;
  // Called to unregister a component with the backend
  virtual void unregisterComponent(Component* component) = 0;

 protected:
  // A pointer to the app in which this backend is running
  Application* app() const { return app_; }

 private:
  friend class Backend;

  Application* app_;
};

// Base class for a component backend for a specific type
template <typename T>
class ComponentBackend : public ComponentBackendBase {
 public:
  virtual ~ComponentBackend() {}

  const char* component_name() const {
    return ComponentName<T>::TypeName();
  }
  void registerComponent(Component* component) override {
    T* ptr = dynamic_cast<T*>(component);
    ASSERT(ptr != nullptr, "Wrong component type");
    registerComponent(ptr);
  }
  void unregisterComponent(Component* component) override {
    T* ptr = dynamic_cast<T*>(component);
    ASSERT(ptr != nullptr, "Wrong component type");
    unregisterComponent(ptr);
  }

 protected:
  virtual void registerComponent(T* component) {}
  virtual void unregisterComponent(T* component) {}
};

}  // namespace alice
}  // namespace isaac
