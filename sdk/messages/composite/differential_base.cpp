/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "differential_base.hpp"

namespace isaac {
namespace messages {

composite::Schema DifferentialBaseTPVASchema() {
  return composite::Schema({
      composite::Quantity::Scalar("time", composite::Measure::kTime),
      composite::Quantity::Vector("base", composite::Measure::kPosition, 2),
      composite::Quantity::Scalar("base", composite::Measure::kRotation),
      composite::Quantity::Scalar("base", composite::Measure::kSpeed),
      composite::Quantity::Scalar("base", composite::Measure::kAngularSpeed),
      composite::Quantity::Scalar("base", composite::Measure::kAcceleration),
      composite::Quantity::Scalar("base", composite::Measure::kAngularAcceleration),
  });
}

composite::Schema DifferentialBaseTPVSchema() {
  return composite::Schema({
      composite::Quantity::Scalar("time", composite::Measure::kTime),
      composite::Quantity::Vector("base", composite::Measure::kPosition, 2),
      composite::Quantity::Scalar("base", composite::Measure::kRotation),
      composite::Quantity::Scalar("base", composite::Measure::kSpeed),
      composite::Quantity::Scalar("base", composite::Measure::kAngularSpeed),
  });
}

}  // namespace messages
}  // namespace isaac
