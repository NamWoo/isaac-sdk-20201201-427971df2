/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/math/gems/optimization/particle_swarm_optimization.hpp"

#include <cmath>

#include "engine/core/constants.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(Filters, ParticleSwarmOptimization) {
  std::default_random_engine rng;

  ParticleSwarmOptimization<Vector2d,2,double> pso;
  pso.num_particles = 100;
  pso.omega = 0.7;
  pso.phi = Vector2d{0.5, 0.5};
  pso.phi_p_to_g = 1.0;
  pso.generator = [&] {
    std::uniform_real_distribution<double> range(-5.0, +5.0);
    return Vector2d{range(rng), range(rng)};
  };
  pso.evaluator = [](const Vector2d& state) {
    // The Ackeley function
    const double x = state.x();
    const double y = state.y();
    return -20.0*std::exp(-0.2*std::sqrt(0.5*(x*x + y*y)))
        - std::exp(0.5*(std::cos(TwoPi<double>*x) + std::cos(TwoPi<double>*y)))
        + std::exp(1.0) + 20.0;
  };
  pso.exp = [](const Vector2d& x, const Vector2d& d) {
    return x + d;
  };
  pso.log = [](const Vector2d& x, const Vector2d& y) {
    return y - x;
  };

  pso.initialize();
  for (int i=0; i<100; i++) {
    pso.step(rng);
  }

  EXPECT_NEAR(pso.best_state().x(), 0.0, 0.01);
  EXPECT_NEAR(pso.best_state().y(), 0.0, 0.01);
  EXPECT_NEAR(pso.best_cost(), 0.0, 0.1);
}

}  // namespace isaac
