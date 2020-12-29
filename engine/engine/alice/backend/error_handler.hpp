/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

namespace google_breakpad {
class ExceptionHandler;
}  // namespace google_breakpad

namespace isaac {
namespace alice {

// Handles various errors including panics and segfaults and prints stack traces
class ErrorHandler  {
 public:
  ErrorHandler();
  ~ErrorHandler();

  // The directory in which minidumps will be saved
  void setMinidumpDirectory(const std::string& directory);

 private:
  // breakpad crash handler
  std::unique_ptr<google_breakpad::ExceptionHandler> exception_handler_;
};

}  // namespace alice
}  // namespace isaac
