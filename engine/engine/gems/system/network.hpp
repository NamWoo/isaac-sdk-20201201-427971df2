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
namespace system {

// Populates the ip parameter with the ip corresponding to the assigned parameter
bool GetIpForAdapter(const std::string& adapter, std::string& ip);

}  // namespace system
}  // namespace isaac
