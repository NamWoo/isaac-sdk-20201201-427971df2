/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/sensor_certification/evaluators/Evaluator.hpp"

namespace isaac {
namespace evaluators {

void Evaluator::reportTimeoutMessage() {
  reportMessage("Test timed out after %f seconds\n", getTimeoutSeconds());
}

void Evaluator::checkTimeout() {
  if (end_timestamp_ == std::nullopt) {
    end_timestamp_ = getTickTimestamp() + (getTimeoutSeconds() * 1e9);
  }

  if (getTickTimestamp() >= end_timestamp_.value()) {
    reportTimeoutMessage();
    reportFailure();
  }
}

}  // namespace evaluators
}  // namespace isaac
