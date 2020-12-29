/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/math/exponential_moving_average.hpp"

#include <random>

#include "gtest/gtest.h"
#include "engine/gems/math/pose_utils.hpp"

namespace isaac {
namespace math {

TEST(ExponentialMovingAverage, Scalar) {
  std::mt19937 rng;
  ExponentialMovingAverage<double> ema(2.0);
  std::uniform_real_distribution<double> obs_dis(4.0, 6.0);
  std::uniform_real_distribution<double> time_dis(0.001, 0.01);
  // Initialize with some value (the beginning is expected to be noisy)
  double time = 0.0;
  while (time < 4.0) {
    time += time_dis(rng);
    ema.add(obs_dis(rng), time);
  }
  while (time < 50.0) {
    time += time_dis(rng);
    EXPECT_NEAR(ema.add(obs_dis(rng), time), 5.0, 0.3);
  }
}

TEST(ExponentialMovingAverage, Pose2) {
  std::mt19937 rng;
  ExponentialMovingAverage<Pose2d> ema(1.0);
  std::uniform_real_distribution<double> xy_dist(2.0, 4.0);
  std::uniform_real_distribution<double> angle_dist(0.5, 1.5);
  std::uniform_real_distribution<double> time_dis(0.001, 0.01);
  // Initialize with some value (the beginning is expected to be noisy)
  double time = 0.0;
  while (time < 4.0) {
    time += time_dis(rng);
    const Pose2d observation = Pose2d::FromXYA(xy_dist(rng), xy_dist(rng), angle_dist(rng));
    ema.add(observation, time);
  }
  while (time < 50.0) {
    time += time_dis(rng);
    const Pose2d observation = Pose2d::FromXYA(xy_dist(rng), xy_dist(rng), angle_dist(rng));
    const Pose2d smoothed = ema.add(observation, time);
    ASSERT_NEAR(smoothed.translation.x(), 3.0, 0.3);
    ASSERT_NEAR(smoothed.translation.y(), 3.0, 0.3);
    ASSERT_NEAR(smoothed.rotation.angle(), 1.0, 0.2);
  }
}

TEST(ExponentialMovingAverage, Pose3) {
  std::mt19937 rng;
  Vector4d sigma;
  sigma[0] = 1.0;
  sigma[1] = 1.0;
  sigma[2] = 1.0;
  sigma[3] = 0.5;
  ExponentialMovingAverage<Pose3d> ema(1.0);
  std::uniform_real_distribution<double> time_dis(0.001, 0.01);
  // Initialize with some value (the beginning is expected to be noisy)
  double time = 0.0;
  while (time < 4.0) {
    time += time_dis(rng);
    const Pose3d observation = PoseNormalDistribution(sigma, rng);
    ema.add(observation, time);
  }
  while (time < 50.0) {
    time += time_dis(rng);
    const Pose3d observation = PoseNormalDistribution(sigma, rng);
    const Pose3d smoothed = ema.add(observation, time);
    ASSERT_NEAR(smoothed.translation.x(), 0.0, 0.7);
    ASSERT_NEAR(smoothed.translation.y(), 0.0, 0.7);
    ASSERT_NEAR(smoothed.translation.z(), 0.0, 0.7);
    ASSERT_NEAR(smoothed.rotation.angle(), 0.0, 0.35);
  }
}

TEST(ExponentialMovingAverageRate, normal_usage) {
  std::mt19937 rng;
  ExponentialMovingAverageRate<double> ema(2.0);
  std::normal_distribution<double> flow_dis(100.0, 10.0);
  std::uniform_real_distribution<double> time_dis(0.0, 0.1);
  // Initialize with some value (the beginning is expected to be noisy)
  double time = 0.0;
  while (time < 10.0) {
    const double dt = time_dis(rng);
    time += dt;
    ema.add(flow_dis(rng) * dt, time);
  }
  while (time < 50.0) {
    const double dt = time_dis(rng);
    time += dt;
    EXPECT_NEAR(ema.add(flow_dis(rng) * dt, time), 100.0, 10.0);
  }
  while (time < 120.0) {
    const double dt = time_dis(rng);
    time += dt;
    ema.updateTime(time);
  }
  EXPECT_NEAR(ema.rate(), 0.0, 1e-2);
}

TEST(ExponentialMovingAverageRate, low_frequency) {
  std::mt19937 rng;
  ExponentialMovingAverageRate<double> ema(2.0);
  std::normal_distribution<double> flow_dis(100.0, 5.0);
  std::uniform_real_distribution<double> time_dis(0.5, 1.5);
  // Initialize with some value (the beginning is expected to be noisy)
  double time = 0.0;
  while (time < 20.0) {
    const double dt = time_dis(rng);
    time += dt;
    ema.add(flow_dis(rng) * dt, time);
  }
  while (time < 100.0) {
    const double dt = time_dis(rng);
    time += dt;
    EXPECT_NEAR(ema.add(flow_dis(rng) * dt, time), 100.0, 20.0);
  }
  while (time < 120.0) {
    const double dt = time_dis(rng);
    time += dt;
    ema.updateTime(time);
  }
  EXPECT_NEAR(ema.rate(), 0.0, 1e-2);
}

}  // namespace math
}  // namespace isaac
