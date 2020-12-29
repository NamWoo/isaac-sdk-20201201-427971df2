/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "error_handler.hpp"

#include <cxxabi.h>
#include <execinfo.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "client/linux/handler/exception_handler.h"

namespace isaac {
namespace alice {

namespace {

// Extracts the demangled function name from a backtrace line
// If successful `demangled` will be reallocated to contain the necessary size.
// Returns the a pointer with the demangled function name; or nullptr if not successful.
char* DemangleBacktraceLine(const char* text, char** demangled, size_t* demangled_size) {
  // Strings from backtrace have the form A(X+B) [P] with A,X,B,P strings
  // X is the mangled function name which we want to demangle to create a better message
  const char* p1 = std::strchr(text, '(') + 1;
  const char* p2 = std::strchr(p1, '+');
  if (p1 == nullptr || p2 == nullptr) {
    return nullptr;
  }
  // Copy mangled name as null-terminated string to a new buffer
  const size_t mangled_size = p2 - p1;
  char* mangled = reinterpret_cast<char*>(std::malloc(mangled_size + 1));  // +1 for null terminator
  if (mangled == nullptr) {
    return nullptr;
  }
  std::strncpy(mangled, p1, mangled_size);
  mangled[mangled_size] = 0;  // we reserved one more
  // demangle the name
  int status;
  char* result = abi::__cxa_demangle(mangled, *demangled, demangled_size, &status);
  if (status != 0) {
    result = nullptr;
  } else {
    *demangled = result;
  }
  std::free(mangled);
  return result;
}

// Crash handler call back function
static bool OnMinidump(const google_breakpad::MinidumpDescriptor& descriptor, void* context,
                       bool succeeded) {
  // Print header
  std::fprintf(stderr, "\033[1;31m");
  std::fprintf(stderr, "====================================================================================================\n");  // NOLINT
  std::fprintf(stderr, "|                            Isaac application terminated unexpectedly                             |\n");  // NOLINT
  std::fprintf(stderr, "====================================================================================================\n");  // NOLINT
  std::fprintf(stderr, "\033[0m");

  // Print the stacktrace with demangled function names (if possible)
  void* array[32];
  const size_t size = backtrace(array, sizeof(array));
  char** ptr = backtrace_symbols(array, size);
  size_t demangled_size = 256;
  char* demangle_buffer = reinterpret_cast<char*>(std::malloc(demangled_size));
  for (size_t i = 0; i < size; i++) {
    char* buffer = DemangleBacktraceLine(ptr[i], &demangle_buffer, &demangled_size);
    std::fprintf(stderr, "\033[1m#%02ld\033[0m ", i + 1);
    if (buffer == nullptr) {
      std::fprintf(stderr, "\033[2m%s\033[0m\n", ptr[i]);
    } else {
      std::fprintf(stderr, "%s \033[2m%s\033[0m\n", buffer, ptr[i]);
    }
  }
  std::free(demangle_buffer);

  // Print footer with mention to minidump
  std::fprintf(stderr, "\033[1;31m");
  std::fprintf(stderr, "====================================================================================================\n");  // NOLINT
  std::fprintf(stderr, "Minidump written to: %s\n", descriptor.path());
  std::fprintf(stderr, "\033[0m");
  return succeeded;
}

}  // namespace

ErrorHandler::ErrorHandler() {
  // Create the breakpad exception handler
  exception_handler_ = std::make_unique<google_breakpad::ExceptionHandler>(
      google_breakpad::MinidumpDescriptor("/tmp"), nullptr, OnMinidump, nullptr, true, -1);
}

ErrorHandler::~ErrorHandler() {}

void ErrorHandler::setMinidumpDirectory(const std::string& directory) {
  exception_handler_->set_minidump_descriptor(google_breakpad::MinidumpDescriptor(directory));
}

}  // namespace alice
}  // namespace isaac
