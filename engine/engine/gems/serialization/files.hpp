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
#include <vector>

namespace isaac {
namespace serialization {

// Read in a binary file at file_path into buffer.
// The contents of buffer will be overwritten by this call.
// This function only returns true if the file was read successfully.
bool ReadEntireBinaryFile(const std::string& file_path, std::vector<char>& buffer);

// Reads an entire text file
bool ReadEntireTextFile(const std::string& file_path, std::string& text);

// Reads the text file and returns a vector of all lines.
// The empty lines are discarded and the function returns true only if the file
// was read succesfully and not empty.
bool ReadTextFileLines(const std::string& file_path, std::vector<std::string>& lines);

}  // namespace serialization
}  // namespace isaac
