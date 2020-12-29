/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "dummy_pose2d_codelets.hpp"

#include <cmath>

#include "messages/math.hpp"

namespace isaac {
namespace dummy {

void DummyPose2dProducer::start() {
  tickPeriodically(0.1);
}

void DummyPose2dProducer::tick() {
  auto pose_builder = tx_pose().initProto();
  auto pose2d = Pose2d{SO2d::FromAngle(DegToRad(60.0)), Vector2d{3.0, -2.1}};
  isaac::ToProto(pose2d, pose_builder);
  tx_pose().publish();
}

void DummyPose2dConsumer::start() {
  tickOnMessage(rx_pose());
}

void DummyPose2dConsumer::tick() {
  auto pose_reader = rx_pose().getProto();
  auto pose2d = FromProto(pose_reader);
  const double error_translation = (pose2d.translation - Vector2d{3.0, -2.1}).norm();
  // we use 1e-14 because of the loss in precision in python
  if (std::abs(error_translation) > 1e-14 ||
      std::abs(RadToDeg(pose2d.rotation.angle()) - 60.0) > 1e-14) {
    throw;  // throw an exception to notify python that the test fails
  }
}

void DummyPose2dConsumer::stop() {
  if (getTickCount() <= get_threshold()) {
    LOG_ERROR("%d <= %d", getTickCount(), get_threshold());
    throw;  // throw an exception to notify python that the test fails
  }
}

}  // namespace dummy
}  // namespace isaac
