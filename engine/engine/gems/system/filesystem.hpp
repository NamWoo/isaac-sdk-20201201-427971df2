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

#include "engine/core/optional.hpp"

namespace isaac {

// test that path directory exist and that the user has read access to it
bool IsValidReadDirectory(const std::string& path);

// test that path directory exist and that the user has write access to it
bool IsValidWriteDirectory(const std::string& path);

// test that path directory exist
bool IsValidDirectory(const std::string& path);

// get file size. Returns std::nullopt if file access fails.
std::optional<size_t> GetFileSize(const std::string& path);

}  // namespace isaac
