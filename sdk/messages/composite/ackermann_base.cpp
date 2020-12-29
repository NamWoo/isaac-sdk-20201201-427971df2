/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ackermann_base.hpp"

namespace isaac {
namespace messages {

composite::Schema AckermannBaseStateSchema() {
  return composite::Schema({
      composite::Quantity::Vector("body", composite::Measure::kPosition, 2),
      composite::Quantity::Scalar("body", composite::Measure::kRotation),
      composite::Quantity::Scalar("steering", composite::Measure::kPosition),
      composite::Quantity::Scalar("body", composite::Measure::kSpeed),
      composite::Quantity::Scalar("body", composite::Measure::kAcceleration)
  });
}

composite::Schema AckermannBaseControlSchema() {
  return composite::Schema({
      composite::Quantity::Vector("body", composite::Measure::kAcceleration, 1),
      composite::Quantity::Vector("steering", composite::Measure::kPosition, 1)
  });
}

}  // namespace messages
}  // namespace isaac
