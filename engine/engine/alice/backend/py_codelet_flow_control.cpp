/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "py_codelet_flow_control.hpp"

#include <condition_variable>
#include <mutex>
#include <string>

#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {
namespace {
constexpr char kStopJob[] = "py_stop";
}  // namespace

// By standard spurious wakeup could happen for conditional variable.
// All waitings on conditional variable have to check for spurious wakeup:
// https://en.cppreference.com/w/cpp/thread/condition_variable

std::string PyCodeletFlowControl::StateToString(PyCodeletFlowControl::State state) {
  switch (state) {
    case PyCodeletFlowControl::State::kIdle:
      return "kIdle";
    case PyCodeletFlowControl::State::kPythonWaiting:
      return "kPythonWaiting";
    case PyCodeletFlowControl::State::kPythonWorking:
      return "kPythonWorking";
    case PyCodeletFlowControl::State::kStopped:
      return "kStopped";
    default:
      PANIC("Bad State %d", state);
  }
}

PyCodeletFlowControl::PyCodeletFlowControl() {
  state_ = State::kStopped;
}

void PyCodeletFlowControl::start() {
  std::unique_lock<std::mutex> lock(m_);

  switch (state_) {
    case State::kPythonWaiting:
    case State::kIdle:
      return;
    case State::kStopped:
      state_ = State::kIdle;
      return;
    default:
      PANIC("Bad State %s", StateToString(state_).c_str());
  }
}

std::optional<std::string> PyCodeletFlowControl::pythonWaitForJob() {
  std::unique_lock<std::mutex> lock(m_);

  switch (state_) {
    case State::kIdle:
      state_ = State::kPythonWaiting;
      cv_.notify_one();
      while (state_ == State::kPythonWaiting) {
        cv_.wait(lock);
      }
      ASSERT(state_ == State::kPythonWorking || state_ == State::kStopped, "Illegal state %s.",
             StateToString(state_).c_str());
      break;
    case State::kStopped:
      return std::nullopt;
    default:
      PANIC("Bad State %s", StateToString(state_).c_str());
  }
  return current_job_;  // returns the job that the python thread should perform
}

void PyCodeletFlowControl::pythonJobFinished() {
  std::unique_lock<std::mutex> lock(m_);
  switch (state_) {
    case State::kStopped:
      // nothing needs to be done during tearing down
      break;
    case State::kPythonWorking:
      state_ = State::kIdle;
      counter_++;
      current_job_ = "";
      cv_.notify_one();  // notifies the cpp thread that it finishes
      return;
    default:
      PANIC("Bad State %s", StateToString(state_).c_str());
  }
}

bool PyCodeletFlowControl::cppDelegateJob(const std::string& job) {
  std::unique_lock<std::mutex> lock(m_);
  switch (state_) {
    case State::kIdle:
      // Waits until python side is ready
      while (state_ == State::kIdle) {
        cv_.wait(lock);
      }
      ASSERT(state_ == State::kPythonWaiting, "Illegal state %s.", StateToString(state_).c_str());
      break;
    case State::kPythonWaiting:
      break;
    default:
      PANIC("Bad State %s", StateToString(state_).c_str());
  }
  current_job_ = job;
  state_ = State::kPythonWorking;
  cv_.notify_one();

  const int64_t cur_counter = counter_;
  while (state_ != State::kIdle && (state_ != State::kPythonWaiting || cur_counter == counter_)) {
    cv_.wait(lock);
  }

  return true;
}

void PyCodeletFlowControl::stop() {
  std::unique_lock<std::mutex> lock(m_);
  switch (state_) {
    case State::kIdle:
      // Waits until python side is ready
      while (state_ == State::kIdle) {
        cv_.wait(lock);
      }
      ASSERT(state_ == State::kPythonWaiting, "Illegal state %s.", StateToString(state_).c_str());
      break;
    case State::kPythonWaiting:
      break;
    default:
      PANIC("Bad State %s", StateToString(state_).c_str());
  }
  // Triggers python thread but doesn't wait for finish,
  // as cpp side is not expecting any result from python side.
  current_job_ = kStopJob;
  state_ = State::kStopped;
  cv_.notify_one();
}

}  // namespace alice
}  // namespace isaac
