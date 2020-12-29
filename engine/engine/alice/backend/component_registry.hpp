/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <map>
#include <set>
#include <string>
#include <type_traits>

#include "engine/core/assert.hpp"
#include "engine/core/singleton.hpp"

namespace isaac {
namespace alice {

class Codelet;
class Component;

// Can be used to get the name of the type and the name of the base type for a component type.
// Synopsis:
//   static constexpr char const* BaseName();  // the typename of the component base class
//   static constexpr char const* TypeName();  // the typename of the component
template <typename T>
struct ComponentName;

// Central register which knows about all components and can create them from their name
class ComponentRegistry {
 public:
  // Registers
  template <typename T>
  int add(const char* type_name) {
    ASSERT(factories_.find(type_name) == factories_.end(),
           "Can not register component twice (typename: '%s')", type_name);
    factories_[std::string(type_name)] = [] {
      T* component = new T();
      component->base_name_ = ComponentName<T>::BaseName();
      component->type_name_ = ComponentName<T>::TypeName();
      return component;
    };
    return counter_++;
  }

  // Creates a codelet with the given name
  Component* create(const std::string& type_name) {
    auto it = factories_.find(type_name);
    ASSERT(it != factories_.end(), "Component with typename '%s' not registered",
           type_name.c_str());
    return it->second();
  }

  // Gets a list of typenames of all registered components
  std::set<std::string> getNames() const {
    std::set<std::string> names;
    for (const auto& kvp : factories_) {
      names.insert(kvp.first);
    }
    return names;
  }

 private:
  std::map<std::string, std::function<Component*()>> factories_;
  int counter_ = 0;
};

namespace component_registry_details {

template <typename T>
struct ComponentRegistryInit {
  static int r;
};

template <typename T>
int ComponentRegistryInit<T>::r =
    Singleton<ComponentRegistry>::Get().add<T>(ComponentName<T>::TypeName());

template <typename T>
struct CheckThatNotCodelet;

}  // namespace component_registry_details

}  // namespace alice
}  // namespace isaac

#define ISAAC_ALICE_REGISTER_COMPONENT_NAME(TYPE, TYPENAME, BASENAME) \
  namespace isaac { namespace alice { \
    template <> struct ComponentName<TYPE> { \
      static constexpr char const* TypeName() { return TYPENAME; } \
      static constexpr char const* BaseName() { return BASENAME; } \
    }; \
  }} \

#define ISAAC_ALICE_REGISTER_COMPONENT_FACTORY(TYPE) \
  namespace isaac { namespace alice { namespace component_registry_details { \
    template struct ComponentRegistryInit<TYPE>; \
  }}} \

// Used to register a base class for components (internal only)
#define ISAAC_ALICE_REGISTER_COMPONENT_BASE(TYPE) \
  ISAAC_ALICE_REGISTER_COMPONENT_NAME(TYPE, #TYPE, #TYPE)

// Used to register a component which can be created (internal only)
#define ISAAC_ALICE_REGISTER_COMPONENT(TYPE) \
  ISAAC_ALICE_REGISTER_COMPONENT_NAME(TYPE, #TYPE, #TYPE) \
  ISAAC_ALICE_REGISTER_COMPONENT_FACTORY(TYPE) \
  namespace isaac { namespace alice { namespace component_registry_details { \
    template <> struct CheckThatNotCodelet<TYPE> { \
      static_assert(!std::is_convertible<TYPE*, isaac::alice::Codelet*>::value, \
                    "Don't use ISAAC_ALICE_REGISTER_COMPONENT for a codelet. " \
                    "Use ISAAC_ALICE_REGISTER_CODELET instead."); \
    }; \
  }}} \

// Registers the name of a codelet. Must be called in the header file of the codelet.
#define ISAAC_ALICE_REGISTER_CODELET_NAME(TYPE) \
  ISAAC_ALICE_REGISTER_COMPONENT_NAME(TYPE, #TYPE, "isaac::alice::Codelet")

// Registers a codelet to the component factory. Must be called in the source file of the codelet.
#define ISAAC_ALICE_REGISTER_CODELET_FACTORY(TYPE) \
  ISAAC_ALICE_REGISTER_COMPONENT_FACTORY(TYPE)

// @deprecated
// Old macro to register codelets. Instead of calling this in the codelet header file use the new
// macro `ISAAC_ALICE_REGISTER_CODELET_NAME`. Additionally call the new macro
// `ISAAC_ALICE_REGISTER_CODELET_FACTORY` in the codelet source file.
#define ISAAC_ALICE_REGISTER_CODELET(TYPE) \
  ISAAC_ALICE_REGISTER_CODELET_NAME(TYPE) \
  ISAAC_ALICE_REGISTER_CODELET_FACTORY(TYPE)
