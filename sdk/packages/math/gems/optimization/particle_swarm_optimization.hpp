/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <vector>

#include "engine/core/math/types.hpp"

namespace isaac {

// Particle swarm global optimization algorithm on N-dimensional state space
//
// This is a global optimization algorithm, meaning that it does not need derivatives, but it also
// does not provide guarantees that it will find a solution. The choice of parameters will have
// a strong influence on convergence and quality of the solution.
//
// Particle swarm optimization uses a "swarm" of particles which move over the state space and try
// to find the overall best state under a given objective function. Particles track the current
// best state per particle and the overall best, and are attracted by this states in their
// movement.
//
// The algorithm uses the function generator to initially seed particles, and the function
// evaluator to evaluate the cost for each particle. This implementation tries to find a local
// minimum, thus the state with the lowest possible cost.
//
// This implementation allows the usage of an arbitrary, non-Euclidean state space and runs particle
// update operations on the Euclidean tangent space. Operations to switch between state space and
// tangent space are provided as the log and exp functions.
template <typename State, int N, typename K>
class ParticleSwarmOptimization {
 public:
  // Tangent state space can be represented by an N-dimensional vector
  using Tangent = Vector<K, N>;

  // In the particle swarm algorithm each particle tracks the current state of the particle and
  // the best state found so far together with the corresponding costs. Additionally a velocity
  // is stored which tracks the movement of the particle through the state space.
  struct Particle {
    State current_state;
    K current_cost;
    State best_state;
    K best_cost;
    Tangent velocity;
  };

  // The number of particles used
  int num_particles;
  // The omega parameter dampens the velocity and should be in the range [0,1]. The higher the
  // value the more erratic particles are using.
  K omega;
  // The phi parameter influences how much particles are attracted the to current best particle
  // state and the overall best state. The value should be positive and related to the resolution
  // and size of the state space. Values can be specified for each state dimension independently.
  Tangent phi;
  // An additional scale factor for phi when used in combination with the overall best state.
  K phi_p_to_g;

  // A functor which is used to seed particles randomly in the state space.
  std::function<State()> generator;
  // A functor which gives a cost to a state. The smaller the better.
  std::function<K(const State&)> evaluator;

  // The exponential and logarithmic maps are used to move from the state space to the tangent
  // space:
  // y = exp(x,d): start at x on the manifold, go into direction d on tangent space, and project
  std::function<State(const State&, Tangent)> exp;
  // d = log(x,y): compute tangent when going from x to y
  std::function<Tangent(const State&, const State&)> log;

  // Removes all particles
  void clear() { particles_.clear(); }
  // Initialize the algorithm by seeding particles
  void initialize();

  // Perform a single step of the PSO algorithm
  // This function samples new particles until `break_test` returns false. `break_test` is called
  // with the number of particles sampled so far. When step stops, the number of sampled particles
  // is returned. The provided random number engine `rng` is used to create necessary randomness.
  template <typename F, typename RandomEngine>
  size_t step(F break_test, RandomEngine& rng);

  template <typename RandomEngine>
  void step(RandomEngine& rng);

  // The state with the lowest cost so far
  const State& best_state() const { return best_state_ever_; }
  // The cost of the best state so far
  K best_cost() const { return best_cost_ever_; }
  // A list of all particles
  const std::vector<Particle>& particles() const { return particles_; }

 private:
  State best_state_ever_;
  K best_cost_ever_;
  std::vector<Particle> particles_;
  size_t step_index_;
};

}  // namespace isaac

#include "ParticleSwarmOptimization.tpp"
