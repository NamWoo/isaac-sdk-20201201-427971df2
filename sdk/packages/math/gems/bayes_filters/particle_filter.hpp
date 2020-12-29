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

namespace isaac {

// A particle filter uses a set of states to track an unknown state step over time.
// Prediction and observation models are used to define the behavior of the filter.
template <typename State, typename Score = double>
class ParticleFilter {
 public:
  // A particle consists of a state and the corresponding score
  struct Particle {
    State state;
    Score score;
  };

  // Adds a new particle
  void addParticle(State state, Score score);
  void addParticle(const Particle& particle);

  // Gets the particle with the highest score
  const Particle& getMaxScoreParticle() const;
  // Gets the score of the particle with the hightest score
  Score getMaxScore() const;
  // Gets a list of particles
  const std::vector<Particle>& particles() const { return particles_; }

  // Iterates over all particles in the upper percentile
  template <typename F>
  void iterateBest(double percentile, F f);

  // Removes all particles
  void clear();

  // Runs a prediction step on every particle using the given prediction model
  // The prediction model gives an estimate of the system state for the next step.
  void predict(std::function<void(State&)> prediction_model);

  // Runs an observation step on every particle using the given observation model. The observation
  // model computes a score for a system state representing how well it matches an observation. The
  // higher the score the better the match.
  // The given callback is used to compute the current score for each particle. The current score
  // is mixed with previous calls to this function until a call to `resetScores` or
  // `resampleAndReset`. Particles
  void observe(std::function<Score(const State&)> observation_model);

  // Resamples the particle set to the desired number of particles
  template <typename RandomEngine>
  void resample(size_t count, RandomEngine& rng);

  // Recomputes scores based on the current and the overall maximum score
  void wrangleScores(std::function<void(Score&, Score)> f);
  // Sets all scores to the same value
  void resetScores(Score new_score);

  // Resamples particle set to same size and resets all scores to 1
  template <typename RandomEngine>
  void resampleAndReset(RandomEngine& rng);

 private:
  void updateMaxScoreParticle(size_t index);

  std::vector<Particle> particles_;
  ssize_t max_score_index_ = -1;
  int num_observations_ = 0;
};

}  // namespace isaac

#include "particle_filter.tpp"
