/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <condition_variable>
#include <mutex>
#include <string>

#include "engine/core/optional.hpp"

namespace isaac {
namespace alice {

// PyCodeletFlowControl helps (C++ and Python) PyCodelet threads to synchronize.
// PyCodelet is designed to expose Python instances with corresponding functions (e.g., start(),
// tick(), and stop()) as regular C++ Codelet instance. Each C++ PyCodelet instance delegates its
// task to corresponding Python PyCodelet instance which is single-threaded.

// Interactions:
// 1) Python thread
//     i) calls pythonWaitForJob() to get a job as string. It blocks until a job is provided by C++
//        thread;
//     ii) execute the job accordingly;
//     iii) calls pythonJobFinished() to notify the C++ thread about finishing;
// 2) C++ thread
//     i) calls cppDelegateJob() to delegate a job as string to the Python instance. It blocks until
//        the Python thread notifies about finishing.
class PyCodeletFlowControl {
 public:
  PyCodeletFlowControl();
  // Cpp thread calls this function to reset state
  void start();
  // Python thread calls this function to retrieve a job (as a std::string). Blocks until
  // a job is delegated or stop() is invoked on cpp side (returns std::nullopt).
  std::optional<std::string> pythonWaitForJob();
  // Python thread calls this function to notify the cpp thread that the job is finished.
  void pythonJobFinished();
  // Cpp thread calls this function to send a job to the python side (as a std::string). The
  // function will block until the job is either completed by python or aborted by other threads.
  bool cppDelegateJob(const std::string& job);
  // Cpp thread calls this function for tearing down. Dispatches `stop()` on Python side but does
  // not wait for its finishing.
  void stop();

 private:
  // condition variable for notifying cpp and python threads
  std::condition_variable cv_;
  // the shared mutex that protects the following member variables.
  std::mutex m_;
  std::string current_job_;  // the current job that the cpp thread tries to send in std::string

  enum class State {
    // all threads are freed
    kIdle = 0,
    // python thread is waiting for cpp
    kPythonWaiting = 1,
    kPythonWorking = 2,
    kStopped = 3
  };
  State state_;
  // Logging helper
  std::string StateToString(State state);

  int64_t counter_ = 0;
};

}  // namespace alice
}  // namespace isaac
