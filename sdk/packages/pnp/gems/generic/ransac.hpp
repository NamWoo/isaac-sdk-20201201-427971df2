/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

// Header for generic RANSAC algorithm with multiple output hypotheses.
// This code is independent of the underlying model and RANSAC scoring method. It can be
// reused for fitting any model (pose, epipolar geometry, homography, line, circle, sphere etc.)

// TODO: Extend hypothesis generation and evaluation scheme to cases when multiple models are
// returned from a single seed, e.g. the 7-point algorithm for fundamental matrix estimation
// may return up to 3 solutions, and the 3-point pose (P3P) problem may have up to 4 solutions.
// In these cases, each of the resulting solution needs to be evaluated using all input.

#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace isaac {
namespace pnp {

// Generic RANSAC hypothesis for any model type.
// T = ModelType  The type of the model to fit to the data.
// sample_size    Number of input data items required to generate a model hypothesis in RANSAC.
// ModelType::operator=() is required in ransac() below.
template <typename T, unsigned sample_size>
struct RansacHypothesis {
  using ModelType = T;                     // Type of the parametric model used in RANSAC
  ModelType model;                         // Hypothesized model
  double score;                            // Score of the model
  std::array<unsigned, sample_size> seed;  // Indices of seed items that generate the hypothesis
  std::vector<unsigned> inliers;           // Indices of all inliers for this hypothesis
};

// Default class for drawing fixed-sized random subsets from a set of elements for RANSAC
// without element repetition per subset (no replacement).
// In unit tests, this can be replaced by a fixed sequence of subsets.
// The subset is called "a sample" and the number of elements in the subset is the "sample size".
// The number of elements in the original set to sample from is called "input size"
template <unsigned sample_size>
class DefaultRansacSampler {
 private:
  // Vector of unique indices of input elements. Indices can be in any order in this vector.
  std::vector<unsigned> indices_;

  // Random engine used by the sampler
  std::mt19937 random_engine_;

 public:
  DefaultRansacSampler(unsigned input_size, unsigned rand_seed) : random_engine_(rand_seed) {
    // The number of items to sample from must be at least the sample size.
    if (input_size < sample_size) {
      input_size = sample_size;
    }

    // Initialize vector of distinct indices.
    // Each index corresponds to an input element that can be sampled.
    indices_.resize(input_size);
    for (unsigned i = 0; i < input_size; i++) {
      indices_[i] = i;
    }
  }

  constexpr unsigned sampleSize() const { return sample_size; }
  unsigned inputSize() const { return indices_.size(); }

  // Draw a sequence of K random samples from N elements without repetition (= no replacement)
  // The probability of each element being selected into the subset is the same.
  // The input index vector must contain the indices 0,1,...,N-1 (all unique) but in any order.
  std::array<unsigned, sample_size> operator()() {
    // Perform K index swaps.
    for (unsigned k = 0; k < sample_size; k++) {
      // Draw a random index uniformly in the range [k, N-1] as k = 0,1,2,...,K
      std::uniform_int_distribution<unsigned> distrib(k, indices_.size() - 1);
      unsigned i = distrib(random_engine_);

      // Swap element k with the selected element.
      std::swap(indices_[k], indices_[i]);
    }
    // Take the first K samples of the shuffled vector as RANSAC seed indices.
    std::array<unsigned, sample_size> subset;
    std::copy(indices_.begin(), indices_.begin() + sample_size, subset.begin());
    return subset;
  }
};

// Debug print utilities (include <iostream>)
template <unsigned sample_size>
std::ostream& operator<<(std::ostream& os, const std::array<unsigned, sample_size>& arr) {
  for (unsigned i : arr) os << " " << arr[i];
  return os;
}
template <typename ModelType, unsigned sample_size>
std::ostream& operator<<(std::ostream& os, const pnp::RansacHypothesis<ModelType, sample_size>& h) {
  os << "score=" << h.score << " inliers=" << h.inliers.size();
  return os;
}

// RANSAC algotrithm returning the top-K hypotheses (avoids repetition of output hypotheses).
// Hypothesis generation and evaluation is provided as an external Algorithm adaptor to keep
// RANSAC code fully generic, i.e. independent of the model and of the scoring method used.
//  Algorithm   Adaptor for a core algorithm to embed into the RANSAC scheme. It provides:
//              (1) Model type to fit (e.g. C++ type of pose or homography or line etc.).
//              (2) Predicate of similarity between two models given the tolerance.
//              (3) Fitting / scoring function that outputs a model hypothesis with inliers listed.
//                  The input is typically minimal or small, sufficient to fit the model.
//              (4) Fitting function for all inliers (typically regression).
//              For example, see pnp::epnp::EpnpRansacAdaptor.
// num_experiments     Number of RANSAC rounds.
// max_top_hypotheses  The capacity of the output list. The top K best hypotheses are kept.
// sampler      Class to draw random samples to fit the model to (see DefaultRansacSampler above).
//
// TODO: expose params controlling how different the top hypotheses should be.
//
template <typename Algorithm, typename RansacSampler>
std::deque<typename Algorithm::ResultType> Ransac(const Algorithm& algorithm,
                                                  unsigned num_experiments,
                                                  unsigned max_top_hypotheses,
                                                  RansacSampler* sampler) {
  // Create empty container of top-K hypotheses.
  std::deque<typename Algorithm::ResultType> top_hypotheses;

  // Make sure the sampler is not null.
  if (sampler == nullptr) return top_hypotheses;

  // Check RANSAC parameters.
  if (num_experiments < 1 || max_top_hypotheses < 1 ||
      sampler->inputSize() != algorithm.inputSize() ||
      sampler->sampleSize() != Algorithm::kSampleSize) {
    return top_hypotheses;
  }

  // Check input of the embedded algorithm.
  if (!algorithm.checkInput()) {
    return top_hypotheses;
  }

  // RANSAC loop using a soft-scoring scheme.
  for (unsigned iter = 0; iter < num_experiments; iter++) {
    // Generate random seed (also known as minimal sampling set).
    std::array<unsigned, Algorithm::kSampleSize> seed = sampler->operator()();
    std::sort(seed.begin(), seed.end());

    // Skip the rest if a hypothesis with this seed is already in the queue.
    bool already_queued = false;
    for (const auto& queued : top_hypotheses) {
      if (seed == queued.seed) {
        already_queued = true;
        break;
      }
    }
    if (already_queued) {
      continue;
    }

    // Generate model hypothesis from the seed and evaluate it.
    auto hypothesis = algorithm.evaluate(seed);

    // Forget this hypothesis if it has less than the minimum number of inliers required.
    // This only happens if the seed is not a minimal set to fit the model.
    if (hypothesis.inliers.size() < Algorithm::kSampleSize) {
      continue;
    }

    // Forget this hypothesis if queue is full and
    // if the worst score in the queue is better than the score of this hypothesis.
    if (top_hypotheses.size() == max_top_hypotheses &&
        top_hypotheses.back().score > hypothesis.score) {
      continue;
    }

    // Avoid adding a hypothesis that is the same as another hypothesis in the queue.
    bool similar_in_queue = false;
    for (const auto& queued : top_hypotheses) {
      similar_in_queue = Algorithm::isSimilar(queued.model, hypothesis.model);
      if (similar_in_queue) {
        break;
      }
    }
    if (similar_in_queue) {
      continue;
    }

    // Insert hypothesis into the queue while keeping the queue ordered.
    auto it = std::upper_bound(top_hypotheses.begin(), top_hypotheses.end(), hypothesis,
                               [](const auto& a, const auto& b) { return a.score > b.score; });
    top_hypotheses.emplace(it, std::move(hypothesis));

    // Keep the top-K queue bounded.
    if (top_hypotheses.size() > max_top_hypotheses) {
      top_hypotheses.pop_back();
    }
  }

  // Re-estimate model of each top hypothesis more accurately from all their inliers.
  for (auto& hypothesis : top_hypotheses) {
    typename Algorithm::ModelType model;
    if (algorithm.refit(hypothesis.inliers, &model)) {
      hypothesis.model = model;
    }
  }

  return top_hypotheses;
}

}  // namespace pnp
}  // namespace isaac
