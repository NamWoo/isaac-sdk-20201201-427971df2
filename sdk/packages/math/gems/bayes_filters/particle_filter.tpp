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
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"

namespace isaac {

template <typename State, typename Score>
void ParticleFilter<State, Score>::addParticle(State state, Score score) {
  addParticle({state, score});
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::addParticle(const Particle& particle) {
  particles_.push_back(particle);
  updateMaxScoreParticle(particles_.size() - 1);
}

template <typename State, typename Score>
const typename ParticleFilter<State, Score>::Particle&
ParticleFilter<State, Score>::getMaxScoreParticle() const {
  ASSERT(!particles_.empty(), "Can not get max score of empty particle set");
  return particles_[max_score_index_];
}

template <typename State, typename Score>
Score ParticleFilter<State, Score>::getMaxScore() const {
  return getMaxScoreParticle().score;
}

template <typename State, typename Score>
template <typename F>
void ParticleFilter<State, Score>::iterateBest(double percentile, F f) {
  std::vector<size_t> indices(particles_.size());
  std::iota(indices.begin(), indices.end(), 0);
  const size_t num_percentile =
      static_cast<size_t>(percentile * static_cast<double>(particles_.size()));
  const auto middle = std::next(indices.begin(), num_percentile);
  std::partial_sort(indices.begin(), middle, indices.end(),
      [this](size_t lhs, size_t rhs) {
        return particles_[lhs].score > particles_[rhs].score;
      });
  for (auto it = indices.begin(); it != middle; ++it) {
    f(particles_[*it]);
  }
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::clear() {
  particles_.clear();
  max_score_index_ = -1;
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::predict(std::function<void(State&)> prediction_model) {
  for (auto& particle : particles_) {
    prediction_model(particle.state);
  }
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::observe(std::function<Score(const State&)> observation_model) {
  // The most common case will be 0 or 1 previous observations. The code below handles these cases
  // separately with higher efficiency.
  max_score_index_ = -1;
  if (num_observations_ == 0) {
    for (size_t i = 0; i < particles_.size(); i++) {
      Particle& particle = particles_[i];
      particle.score = observation_model(particle.state);
      updateMaxScoreParticle(i);
    }
  } else if (num_observations_ == 1) {
    for (size_t i = 0; i < particles_.size(); i++) {
      Particle& particle = particles_[i];
      const Score current_score = observation_model(particle.state);
      const Score new_score = std::sqrt(particle.score * current_score);
      particle.score = new_score;
      updateMaxScoreParticle(i);
    }
  } else {
    const double exponent_1 = num_observations_;
    const double exponent_2 = 1. / (num_observations_ + 1);
    for (size_t i = 0; i < particles_.size(); i++) {
      Particle& particle = particles_[i];
      particle.score = std::pow(
        std::pow(particle.score, exponent_1) * observation_model(particle.state),
        exponent_2);
      updateMaxScoreParticle(i);
    }
  }
  num_observations_++;
}

template <typename State, typename Score>
template <typename RandomEngine>
void ParticleFilter<State, Score>::resample(size_t count, RandomEngine& rng) {
  // keep a copy of the old particles for the algorithm and create a list of new particles to fill
  const size_t num_old_particles = particles_.size();
  if (num_old_particles == 0) {
    ASSERT(count == 0, "Can not resample any particles from an empty particle set", count);
    return;
  }
  // Save old particles before running the resampling wheel
  const Score max_score = getMaxScore();
  std::vector<Particle> old_particles = std::move(particles_);
  particles_ = std::vector<Particle>();
  particles_.reserve(count);
  max_score_index_ = -1;
  // run the resampling wheel to randomly sample the desired number of particles based on score
  std::uniform_int_distribution<size_t> random_index(0, num_old_particles - 1);
  std::uniform_real_distribution<Score> random_delta(0, Score(2.0) * max_score);
  int index = random_index(rng);
  Score beta = Score(0.0);
  for (size_t i = 0; i < count; i++) {
    beta += random_delta(rng);
    while (beta > old_particles[index].score) {
      beta -= old_particles[index].score;
      index = (index + 1) % num_old_particles;
    }
    addParticle(old_particles[index]);
  }
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::wrangleScores(std::function<void(Score&, Score)> f) {
  const Score max_score = getMaxScore();
  max_score_index_ = -1;
  for (size_t i = 0; i < particles_.size(); i++) {
    f(particles_[i].score, max_score);
    updateMaxScoreParticle(i);
  }
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::resetScores(Score new_score) {
  for (auto& particle : particles_) {
    particle.score = new_score;
  }
  // We leave the max score pointer unchanged.
  // Reset observation count
  num_observations_ = 0;
}

template <typename State, typename Score>
template <typename RandomEngine>
void ParticleFilter<State, Score>::resampleAndReset(RandomEngine& rng) {
  resample(particles_.size(), rng);
  resetScores(Score(1));
}

template <typename State, typename Score>
void ParticleFilter<State, Score>::updateMaxScoreParticle(size_t index) {
  if (max_score_index_ == -1 || particles_[index].score > particles_[max_score_index_].score) {
    max_score_index_ = index;
  }
}

}  // namespace isaac
