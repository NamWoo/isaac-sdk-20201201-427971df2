/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "files.hpp"

#include <fstream>
#include <string>
#include <vector>

#include "engine/core/logger.hpp"

namespace isaac {
namespace serialization {

bool ReadEntireBinaryFile(const std::string& file_path, std::vector<char>& buffer) {
  // Open the file in binary mode and seek to the end
  std::ifstream file(file_path, std::ios::binary | std::ios::ate);
  if (!file) {
    return false;
  }
  // Get the size of the file and seek back to the beginning
  const size_t size = file.tellg();
  file.seekg(0);
  // Reserve enough space in the output buffer and read the file contents into it
  buffer.resize(size);
  return static_cast<bool>(file.read(buffer.data(), size));
}

bool ReadEntireTextFile(const std::string& file_path, std::string& text) {
  // Open the file in text and seek to the end
  std::ifstream ifs(file_path, std::ios::ate);
  if (!ifs) {
    return false;
  }
  // Get the size of the file and seek back to the beginning
  const size_t size = ifs.tellg();
  ifs.seekg(0);
  // Reserve enough space in the output buffer and read the file contents into it
  text.resize(size);
  return static_cast<bool>(ifs.read(&text[0], size));
}

bool ReadTextFileLines(const std::string& file_path, std::vector<std::string>& lines) {
  std::ifstream file(file_path);
  if (!file) {
    return false;
  }
  std::string line;
  // Read every line and discard empty lines
  while (std::getline(file, line)) {
    if (line.size() == 0) {
      continue;
    }
    lines.push_back(line);
  }
  return !lines.empty();
}

}  // namespace serialization
}  // namespace isaac
