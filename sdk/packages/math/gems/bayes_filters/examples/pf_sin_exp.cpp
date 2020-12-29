/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <algorithm>
#include <cmath>
#include <thread>

#include "engine/gems/sight/sight.hpp"
#include "packages/math/gems/bayes_filters/particle_filter.hpp"

namespace isaac {

double ObservationModel(double x) {
  return std::exp(2.0 * x);
}

void Main() {
  constexpr int kNumParticles = 50;
  constexpr int kNumIterations = 500;
  constexpr double kDT = 0.02;

  ParticleFilter<double, double> pf;

  for (int i = 0; i < kNumParticles; i++) {
    pf.addParticle(0.0, 0.0);
  }

  double time = 0.0f;

  std::default_random_engine rng;
  std::normal_distribution<double> noise_predict(0.0f, 0.1f);
  std::normal_distribution<double> noise_observe(0.0f, 0.1f);

  for (int i = 0; i < kNumIterations; i++) {
    time += kDT;

    const double actual = std::sin(time);
    sight::Plot("PfExampleSinExp.state.actual", actual);

    const double observation = ObservationModel(actual) + noise_observe(rng);
    sight::Plot("PfExampleSinExp.state.observed", observation);

    pf.predict([&](double& state) {
      state += noise_predict(rng);
    });
    pf.observe([&](double state) {
      const double z = ObservationModel(state);
      const double delta = z - observation;
      return std::exp(-0.5 * delta * delta);
    });
    sight::Plot("PfExampleSinExp.state.estimate", pf.getMaxScoreParticle().state);
    sight::Plot("PfExampleSinExp.score.max", pf.getMaxScoreParticle().score);
    const double min_score = std::min_element(pf.particles().begin(), pf.particles().end(),
        [](const auto& a, const auto& b) { return a.score < b.score; })->score;
    sight::Plot("PfExampleSinExp.score.min", min_score);
    pf.resampleAndReset(rng);

    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.0 / kDT));
  }
}

}  // namespace isaac

int main(int argc, char** argv) {
  isaac::Main();
}
