/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <string>

#include "engine/alice/backend/component_registry.hpp"
#include "engine/alice/component_impl.hpp"

namespace isaac {
namespace alice {

// A type used by various private member function to find certain components.
struct ComponentQuery {
  // If this function returns true the component matches the criteria.
  std::function<bool(Component*)> test;
  // In case of errors this string will be used in the error message.
  std::string message;
};

// Creates a query to check if a component has a certain type
template <typename T>
ComponentQuery CreateComponentTypeQuery() {
  return {
    [](Component* component) { return dynamic_cast<T*>(component) != nullptr; },
    ComponentName<T>::TypeName()
  };
}

}  // namespace alice
}  // namespace isaac
