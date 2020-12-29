/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <random>

namespace isaac {

template <typename State, int N, typename K>
void ParticleSwarmOptimization<State, N, K>::initialize() {
  const size_t target = static_cast<size_t>(num_particles);
  // crop the set if too many particles
  if (target < particles_.size()) {
    particles_.resize(num_particles);
  }
  // add new particles until we have enough
  while (particles_.size() < target) {
    const State p = generator();
    const K current_cost = evaluator(p);
    if (particles_.empty() || current_cost < best_cost_ever_) {
      best_state_ever_ = p;
      best_cost_ever_ = current_cost;
    }
    particles_.push_back(Particle{p, current_cost, p, current_cost, Tangent::Zero()});
  }
  step_index_ = 0;
}

template <typename State, int N, typename K>
template <typename F, typename RandomEngine>
size_t ParticleSwarmOptimization<State, N, K>::step(F break_test, RandomEngine& rng) {
  if (particles_.empty()) {
    return 0;
  }
  std::normal_distribution<K> gaussian;
  size_t count = 0;
  while (break_test(count++)) {
    // advance to the next particle
    step_index_ = step_index_ % particles_.size();
    auto& particle = particles_[step_index_++];
    // compute random steps
    Tangent scale_p, scale_g;
    for (int i = 0; i < N; i++) {
      scale_p[i] = gaussian(rng);
      scale_g[i] = gaussian(rng);
    }
    scale_p = scale_p.array() * phi.array();
    scale_g = scale_g.array() * phi.array() * phi_p_to_g;
    // compute velocity
    particle.velocity =
        omega * particle.velocity
        + Tangent(scale_p.array() * log(particle.current_state, particle.best_state).array()
                + scale_g.array() * log(particle.current_state, best_state_ever_).array());
    // update current state and cost
    particle.current_state = exp(particle.current_state, particle.velocity);
    particle.current_cost = evaluator(particle.current_state);
    // update particle best and overall best
    if (particle.current_cost < particle.best_cost) {
      particle.best_state = particle.current_state;
      particle.best_cost = particle.current_cost;
      if (particle.best_cost < best_cost_ever_) {
        best_state_ever_ = particle.best_state;
        best_cost_ever_ = particle.best_cost;
      }
    }
  }
  return count;
}

template <typename State, int N, typename K>
template <typename RandomEngine>
void ParticleSwarmOptimization<State, N, K>::step(RandomEngine& rng) {
  step([this](size_t count) { return count < particles_.size(); }, rng);
}

}  // namespace isaac
