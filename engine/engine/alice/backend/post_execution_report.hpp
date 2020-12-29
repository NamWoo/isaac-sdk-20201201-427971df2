/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <string>

#include "third_party/nlohmann/json.hpp"

namespace isaac {
namespace alice {

// Creates execution reports as strings from a JSON objects of backend statistics. This code will
// be completely moved out of Isaac Engine in future CL.
std::map<std::string, std::string> StringifyBackendStatistics(const nlohmann::json& json);

}  // namespace alice
}  // namespace isaac
