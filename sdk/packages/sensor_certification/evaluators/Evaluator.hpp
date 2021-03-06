/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/alice_codelet.hpp"

namespace isaac {
namespace evaluators {

// A base class with useful functions common to all evaluators codelets
class Evaluator : public alice::Codelet {
 public:
  // Report string used to send detailed failure/success infomation to pyalice
  ISAAC_PARAM(std::string, report, "");

  // The directory to save any extra report files generated by this test
  ISAAC_PARAM(std::string, report_directory, "/tmp");

  // This parameter is set to true once interactive user setup has been completed. it may not be
  // used by every evaluator
  ISAAC_PARAM(bool, setup_done, false);

 protected:
  // Returns the number of seconds the test should run before reporting failure due to a timeout
  virtual float getTimeoutSeconds() = 0;
  // Reports a detailed message indicating reason for timeout
  virtual void reportTimeoutMessage();
  // Reports failure if the codlet has been running longer than the allowed timeout
  void checkTimeout();
  // Log a message from the evaluator into the report parameter
  template<typename... Types>
  inline void reportMessage(const char* format, const Types&... args) {
    const uint32_t kBufferSize = 4096;
    char buffer[kBufferSize];
    std::snprintf(buffer, kBufferSize, format, args...);
    report_ += buffer;
    set_report(report_);
  }

 private:
  // The time at which the codelet will fail due to timeout
  std::optional<int64_t> end_timestamp_;
  // A list of message logged through the reportMessage function
  std::string report_;
};

}  // namespace evaluators
}  // namespace isaac
