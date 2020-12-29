/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "names.hpp"

#include <string>

#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

namespace {
constexpr char const* kForbiddenNameCharacters = "/";
}  // details

bool CheckValidName(const std::string& name) {
  return name.find_first_of(kForbiddenNameCharacters) == std::string::npos;
}

void AssertValidName(const std::string& name) {
  ASSERT(CheckValidName(name),
         "Name '%s' may not contain any of the following characters: %s",
         name.c_str(), kForbiddenNameCharacters);
}

}  // namespace alice
}  // namespace isaac
