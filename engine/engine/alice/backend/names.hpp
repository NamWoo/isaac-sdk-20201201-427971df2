/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

namespace isaac {
namespace alice {

// Returns true if the given name is valid
// A valid name does not contain any forbidden characters.
bool CheckValidName(const std::string& name);

// Asserts if the given name is not valid
void AssertValidName(const std::string& name);

}  // namespace alice
}  // namespace isaac
