/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>

#include "engine/core/assert.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"

namespace isaac {
namespace math {

namespace exponential_moving_average_details {

// Helper type for ExponentialMovingAverage to add support for a linear space, e.g. scalar or vector
template <typename T>
struct ExponentialMovingAverageHelpers {
  // The state space type
  using value_t = T;
  // A const reference to an element in the state space
  using const_ref_t = const T&;
  // The underlying scalar type for the state space
  using scalar_t = T;
  // Type for tangent vectors to the state space
  using tangent_t = T;
  // The identity element of the state space. e.g. Exp(x, Log(id)) = x
  static value_t Identity() { return 0.; }
  // The manifold exponential map for the state space which "unprojects" an element in the tagent
  // space to the state space.
  static value_t Exp(const_ref_t x, const tangent_t& delta) { return x + delta; }
  // The manifold logarithm map for the state space which projects an element in the state space
  // to the tangent space.
  static tangent_t Log(const_ref_t x, const_ref_t y) { return y - x; }
};

// Helper type for ExponentialMovingAverage to add support for Pose2
template <typename K>
struct ExponentialMovingAverageHelpers<Pose2<K>> {
  using scalar_t = K;
  using value_t = Pose2<K>;
  using const_ref_t = const Pose2<K>&;
  using tangent_t = Vector3<K>;
  static value_t Identity() { return Pose2<K>::Identity(); }
  static tangent_t Log(const_ref_t x, const_ref_t y) { return Pose2Log(x.inverse() * y); }
  static value_t Exp(const_ref_t x, const tangent_t& delta) { return x * Pose2Exp(delta); }
};

// Helper type for ExponentialMovingAverage to add support for Pose3
template <typename K>
struct ExponentialMovingAverageHelpers<Pose3<K>> {
  using scalar_t = K;
  using value_t = Pose3<K>;
  using const_ref_t = const Pose3<K>&;
  using tangent_t = Vector6<K>;
  static value_t Identity() { return Pose3<K>::Identity(); }
  static tangent_t Log(const_ref_t x, const_ref_t y) { return Pose3Log(x.inverse() * y); }
  static value_t Exp(const_ref_t x, const tangent_t& delta) { return x * Pose3Exp(delta); }
};

}  // namespace exponential_moving_average_details

// Computes an exponential moving average for a 2D transformation (Pose2).
// See https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average for details.
// log and exp functions from SE(2) space are used for interpolation.
template <typename T, typename K =
    typename exponential_moving_average_details::ExponentialMovingAverageHelpers<T>::scalar_t>
class ExponentialMovingAverage {
 public:
  using helpers_t = exponential_moving_average_details::ExponentialMovingAverageHelpers<T>;
  using value_t = typename helpers_t::value_t;
  using const_ref_t = typename helpers_t::const_ref_t;
  using tangent_t = typename helpers_t::tangent_t;

  // Creates an exponential moving average with the given smoothing period `lambda`. A large
  // smoothing period results in a smoother output. The ratio of influence of values not older than
  // the smoothing period compared to those older than the smoothing period is approximately 1.7.
  ExponentialMovingAverage(K lamda)
      : is_first_time_(true), current_time_(K(0)), current_value_(helpers_t::Identity()) {
    ASSERT(lamda > K(0), "Lambda must be positive");
    lamda_inv_ = K(1) / lamda;
  }
  ExponentialMovingAverage() : ExponentialMovingAverage(K(1)) {}

  // Adds a new observation for the givne time and updates the current smoothed estimate. New
  // observations advance the internal timeline. An observation with an older time will be
  // discarded by the filter.
  const_ref_t add(const_ref_t value, K time) {
    if (is_first_time_) {
      current_value_ = value;
      current_time_ = time;
      is_first_time_ = false;
      return value;
    }
    const K dt = time - current_time_;
    // We receive a previous measurement, unfortunately we can't incorporate it.
    if (dt <= 0) return current_value_;
    const K weight = K(1) - std::exp(-dt * lamda_inv_);
    const tangent_t delta = helpers_t::Log(current_value_, value);
    current_value_ = helpers_t::Exp(current_value_, weight * delta);
    current_time_ = time;
    return current_value_;
  }

  // Returns current time of the filter. This is the maximum of all time values passed to calls to
  // the `add` function. If `add` was never called this function returns 0.
  K time() const { return current_time_; }

  // Returns the smooth value of the filter computed during the last call to `add`. If `add` was
  // never called this function returns identity.
  const_ref_t value() const { return current_value_; }

 protected:
  K lamda_inv_;
  bool is_first_time_;
  K current_time_;
  value_t current_value_;
};

// Helper function to maintain a moving average of a rate.
// Usage:
//  ExponentialMovingAverageRate(time_window)
//  add(additive_flow, time) -> returns the current best estimaton.
template <class K>
class ExponentialMovingAverageRate {
 public:
  ExponentialMovingAverageRate() : ExponentialMovingAverageRate(K(1)) {}
  ExponentialMovingAverageRate(K lamda)
  : lamda_inv_(K(1) / lamda), is_first_time_(true), current_time_(K(0)), current_rate_(K(0)) {}

  // Adds a new measurement, updates the current value and returns it.
  K add(K flow, K time) {
    if (is_first_time_) {
      current_time_ = time;
      current_rate_ = flow * lamda_inv_;
      is_first_time_ = false;
      return current_rate_;
    }
    const K dt = time - current_time_;
    if (dt <= 0) {
      // We receive a previous measurement, let's just add it.
      current_rate_ += flow * lamda_inv_;
    } else {
      // Decay the rate exponentially
      current_rate_ -= adjustmentFactor(dt) * (dt * current_rate_ - flow);
      current_time_ = time;
    }
    return current_rate_;
  }

  // Updates the last time (decays the current value accordingly).
  void updateTime(K time) {
    const K dt = time - current_time_;
    if (dt <= 0) return;
    current_rate_ *= decayFactor(dt);
    current_time_ = time;
  }

  // Returns the last time we have got an update
  K time() const { return current_time_; }

  // Returns the current estimated rate.
  K rate() const { return current_rate_; }

 protected:
  // Helper function computing the second-order approximation of the function `(1 - exp(-x)) / x`.
  static K Approximation(K x) {
    return K(1) + x * (x * K(1.0 / 6.0) - K(0.5));
  }

  // Decay factor for given time period computed as `exp(-dt/l)`.
  // If dt is small enough we can use the second order approximation.
  K decayFactor(K dt) const {
    const K dx = dt * lamda_inv_;
    return (dx < K(0.1)) ? (K(1) - dx * Approximation(dx)) : std::exp(-dx);
  }

  // Rate adjustment factor for given time period computed as `(1 - exp(dt/l)) / dt`.
  // If dt is small enough we can use the third order approximation.
  K adjustmentFactor(K dt) const {
    const K dx = dt * lamda_inv_;
    return (dx < K(0.1)) ? (Approximation(dx) * lamda_inv_) : ((K(1) - std::exp(-dx)) / dt);
  }

  K lamda_inv_;
  bool is_first_time_;
  K current_time_;
  K current_rate_;
};

}  // namespace math
}  // namespace isaac
