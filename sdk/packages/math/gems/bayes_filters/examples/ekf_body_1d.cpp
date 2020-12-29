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

struct State : state::State<float, 3> {
  ISAAC_STATE_VAR(0, position);
  ISAAC_STATE_VAR(1, speed);
  ISAAC_STATE_VAR(2, acceleration);
};
struct Control : state::State<float, 1> {
  ISAAC_STATE_VAR(0, jerk);
};
struct Observation : state::State<float, 1> {
  ISAAC_STATE_VAR(0, position);
};

auto CreatePredictionModel() {
  return EkfPredictionModel<State, Control>{
      [](State& x, float dt, const Control& u) {
        x.position() += dt * x.speed();
        x.speed() += dt * x.acceleration();
        x.acceleration() += dt * u.jerk();
      },
      [](const State& x, float dt, const Control& u) {
        EkfPredictJacobian<State> J = EkfPredictJacobian<State>::Zero();
        J(State::kI_position, State::kI_position) = 1.0f;
        J(State::kI_position, State::kI_speed) = dt;
        J(State::kI_speed, State::kI_speed) = 1.0f;
        J(State::kI_speed, State::kI_acceleration) = dt;
        J(State::kI_acceleration, State::kI_acceleration) = 1.0f;
        return J;
      },
      [](const State& x) {
        EkfCovariance<State> noise = EkfCovariance<State>::Zero();
        noise(State::kI_position, State::kI_position) = 0.1f;
        noise(State::kI_speed, State::kI_speed) = 0.1f;
        noise(State::kI_acceleration, State::kI_acceleration) = 1.0f;
        return noise;
      }};
}

auto CreateObservationModel() {
  return EkfObservationModel<State, Observation>{
      [](const State& x) {
        Observation z;
        z.position() = x.position();
        return z;
      },
      [](const Observation& lhs, const Observation& rhs) {
        Observation z;
        z.elements = lhs.elements - rhs.elements;
        return z;
      },
      [](const State& x) {
        EkfObserveJacobian<State, Observation> J;
        J(Observation::kI_position, State::kI_position) = 1.0f;
        J(Observation::kI_position, State::kI_speed) = 0.0f;
        J(Observation::kI_position, State::kI_acceleration) = 0.0f;
        return J;
      },
      [](const State& x, const Observation& z) {
        EkfCovariance<Observation> noise;
        noise(Observation::kI_position, Observation::kI_position) = 1.0f;
        return noise;
      }};
}

void Main() {
  constexpr int kNumIterations = 5000;
  constexpr float kDT = 0.05f;

  auto prediction_model = CreatePredictionModel();
  auto observation_model = CreateObservationModel();

  float time = 0.0f;
  State actual;
  actual.position() = 0.0f;
  actual.speed() = 0.0f;
  actual.acceleration() = 0.0f;
  State state = actual;
  Control control;
  EkfCovariance<State> covariance = EkfCovariance<State>::Identity();
  EkfCovariance<State> actual_covariance = EkfCovariance<State>::Identity();

  std::default_random_engine rng;
  std::normal_distribution<float> noise_predict_position(0.0f, 0.1f);
  std::normal_distribution<float> noise_predict_speed(0.0f, 0.1f);
  std::normal_distribution<float> noise_predict_acceleration(0.0f, 1.0f);
  std::normal_distribution<float> noise_observe(0.0f, 1.0f);

  for (int i = 0; i < kNumIterations; i++) {
    time += kDT;
    control.jerk() = -std::cos(time);  // third derivative of std::sin(time)

    prediction_model.predict(actual, actual_covariance, kDT, control);
    actual.position() += noise_predict_position(rng);
    actual.speed() += noise_predict_speed(rng);
    actual.acceleration() += noise_predict_acceleration(rng);
    sight::Plot("EkfExampleRigidBody1D.actual_position", actual.position());

    prediction_model.predict(state, covariance, kDT, control);
    Observation z = observation_model.observe_state(actual);
    z.position() += noise_observe(rng);
    observation_model.observe(state, covariance, z);

    sight::Plot("EkfExampleRigidBody1D.observed_position", z.position());
    sight::Plot("EkfExampleRigidBody1D.estimated_position", state.position());

    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(1.0f / kDT));
  }
}

}  // namespace isaac

int main(int argc, char** argv) {
  isaac::Main();
}
