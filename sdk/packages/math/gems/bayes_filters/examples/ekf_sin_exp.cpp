/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <cmath>
#include <random>
#include <thread>

#include "engine/gems/sight/sight.hpp"
#include "packages/engine_gems/state/state.hpp"
#include "packages/math/gems/bayes_filters/extended_kalman_filter.hpp"

namespace isaac {

using Matrix1f = Matrix<float, 1, 1>;

struct State : state::State<float, 1> {
  ISAAC_STATE_VAR(0, amplitude);
};
struct Observation : state::State<float, 1> {
  ISAAC_STATE_VAR(0, measurement);
};

auto CreatePredictionModel() {
  return EkfPredictionModel<State, EkfNoControlF>{
      [](State& x, float dt, const EkfNoControlF& u) {},
      [](const State& x, float dt, const EkfNoControlF& u) {
        EkfPredictJacobian<State> J;
        J(State::kI_amplitude, State::kI_amplitude) = 1.0f;
        return J;
      },
      [](const State& x) {
        EkfCovariance<State> noise;
        noise(State::kI_amplitude, State::kI_amplitude) = 0.1f;
        return noise;
      }};
}

auto CreateObservationModel() {
  return EkfObservationModel<State, Observation>{
      [](const State& x) {
        Observation z;
        z.measurement() = std::exp(2.0f * x.amplitude());
        return z;
      },
      [](const Observation& lhs, const Observation& rhs) {
        Observation z;
        z.elements = lhs.elements - rhs.elements;
        return z;
      },
      [](const State& x) {
        EkfObserveJacobian<State, Observation> J;
        J(Observation::kI_measurement, State::kI_amplitude) = 2.0f * std::exp(2.0f * x.amplitude());
        return J;
      },
      [](const State& x, const Observation& z) {
        EkfCovariance<Observation> noise;
        noise(Observation::kI_measurement, Observation::kI_measurement) = 0.1f;
        return noise;
      }};
}

void Main() {
  constexpr int kNumIterations = 500;
  constexpr float kDT = 0.02f;

  auto prediction_model = CreatePredictionModel();
  auto observation_model = CreateObservationModel();

  float time = 0.0f;
  State actual;
  actual.amplitude() = 0.0f;
  State state;
  state.amplitude() = 0.0f;
  Matrix1f covariance = Matrix1f::Identity();

  std::default_random_engine rng;
  std::normal_distribution<float> noise_predict(0.0f, 0.1f);
  std::normal_distribution<float> noise_observe(0.0f, 0.1f);

  for (int i = 0; i < kNumIterations; i++) {
    time += kDT;
    actual.amplitude() = std::sin(time);
    sight::Plot("EkfExampleSinExp.actual", actual.amplitude());

    prediction_model.predict(state, covariance, kDT);
    Observation z = observation_model.observe_state(actual);
    z.measurement() += noise_observe(rng);
    observation_model.observe(state, covariance, z);

    sight::Plot("EkfExampleSinExp.observed", z.measurement());
    sight::Plot("EkfExampleSinExp.estimate_lo", state.amplitude() - covariance(0, 0));
    sight::Plot("EkfExampleSinExp.estimate", state.amplitude());
    sight::Plot("EkfExampleSinExp.estimate_up", state.amplitude() + covariance(0, 0));

    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.0f / kDT));
  }
}

}  // namespace isaac

int main(int argc, char** argv) {
  isaac::Main();
}
