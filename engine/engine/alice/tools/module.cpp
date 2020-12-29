/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>
#include <utility>

#include "engine/alice/backend/component_registry.hpp"
#include "engine/alice/tools/gather_component_info.hpp"
#include "engine/gems/serialization/json.hpp"

extern "C" {

// A function to load information about all components in a module. The information is provided as
// a serialized JSON object. The JSON string is copied in the provided buffer `result`, copying at
// most `length` bytes. The total number of bytes necessary to store the object is returned. In case
// `result` is nullptr no copy takes place.
int IsaacGatherComponentInfo(char* result, int length) {
  const auto json = ::isaac::alice::GatherComponentInfo();
  const std::string text = json.dump(2);
  if (result != nullptr && length > 0) {
    std::strncpy(result, text.c_str(), length);
  }
  return text.size() + 1;
}

// Creates a component with the given type name. At the moment this function asserts in case the
// component could not be created.
void* IsaacCreateComponent(const char* type_name) {
  return static_cast<void*>(
      ::isaac::Singleton<::isaac::alice::ComponentRegistry>::Get().create(type_name));
}

}  // extern
