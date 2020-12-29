/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "filesystem.hpp"

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

#include <string>

#include "engine/core/optional.hpp"

namespace isaac {

bool IsValidReadDirectory(const std::string& path) {
  const int read_access = access(path.c_str(), R_OK);
  return IsValidDirectory(path) && read_access == 0;
}

bool IsValidWriteDirectory(const std::string& path) {
  const int write_access = access(path.c_str(), W_OK);
  return IsValidDirectory(path) && write_access == 0;
}

bool IsValidDirectory(const std::string& path) {
  const int dir_exist = access(path.c_str(), F_OK);
  return dir_exist == 0;
}

std::optional<size_t> GetFileSize(const std::string& path) {
  struct stat st;
  if (stat(path.c_str(), &st) != 0) {
    return std::nullopt;
  }

  return std::optional<size_t>(st.st_size);
}
}  // namespace isaac
