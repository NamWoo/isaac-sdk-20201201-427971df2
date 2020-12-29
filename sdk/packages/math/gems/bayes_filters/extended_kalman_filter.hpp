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

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"
#include "packages/engine_gems/state/state.hpp"

namespace isaac {

// This file contains helper classes and functions to implement an extended Kalman filter.
//
// An extended Kalman filter is the non-linear extension of the Kalman filter.
// """A Kalman filter is an algorithm that uses a series of measurements observed over time,
//    containing statistical noise and other inaccuracies, and produces estimates of unknown
//    variables that tend to be more accurate than those based on a single measurement alone, by
//    estimating a joint probability distribution over the variables for each timeframe."""
// See Wikipedia (https://en.wikipedia.org/wiki/Extended_Kalman_filter) for more details.
//
// EkfPredictionModel and EkfObservationModel hold the prediction and observation models of an
// extended Kalman filter. Together they can be used to implement an extended Kalman filter.
// The prediction and observation models are separate to allow an easy recombination of different
// models into different filters, or even dynamically exchange a model in a specific situations.
//
// Kalman filters use the state::State type for state, control and observation vectors. The
// state::State type provides named access to elements in the state vector. This mechanics helps to
// avoid index erros when setting elements in the state vectors or computing Jacobians.

// A special type which can be used when there is no control vector. Identical to state::State<K,0>.
template <typename K>
using EkfNoControl = state::State<K, 0>;
using EkfNoControlF = EkfNoControl<float>;
using EkfNoControlD = EkfNoControl<double>;

// Convenience types to get types for the state covariance matrices and jacobians
template <typename State>
using EkfCovariance = Matrix<typename State::Scalar, State::kDimension, State::kDimension>;
template <typename State>
using EkfPredictJacobian = Matrix<typename State::Scalar, State::kDimension, State::kDimension>;
template <typename State, typename Observation>
using EkfObserveJacobian =
    Matrix<typename State::Scalar, Observation::kDimension, State::kDimension>;

namespace ekf_details {

// Computes A * X * A^t
template <typename K, int N, int M>
Matrix<K, M, M> AXAt(const Matrix<K, N, N>& X, const Matrix<K, M, N>& A) {
  // TODO: Use optimized implementation
  return A * X * A.transpose();
}

// Makes sure that a matrix is symmetric
template <typename Derived>
auto EnforceSymmetry(const Eigen::MatrixBase<Derived>& A) {  // returns a matrix of same type as A
  using K = typename Derived::Scalar;
  static_assert(std::is_floating_point<K>::value, "This function only works for floating points");
  auto A_eval = A.eval();
  return (K(0.5) * (A_eval + A_eval.transpose())).eval();
}

}  // namespace ekf_details

// Prediction model for the Extended Kalman filter
//
// K: type of scalar
// NX: dimension of the state space X (or Eigen::Dynamic for runtime size)
// NU: dimension of the control space U (or Eigen::Dynamic for runtime size)
template <typename X, typename U = EkfNoControl<typename X::Scalar>>
struct EkfPredictionModel {
  using K = typename X::Scalar;
  static_assert(std::is_same<K, typename U::Scalar>::value,
                "Scalar type for state and control must be identical");
  static constexpr int NX = X::kDimension;
  static constexpr int NU = U::kDimension;
  // Various linear algebra types for state vectors and covariance matrices
  using P_t = EkfCovariance<X>;
  using F_t = EkfPredictJacobian<X>;
  using Q_t = EkfCovariance<X>;

  // State transition function f: X x U -> X
  std::function<void(X& x, K dt, const U& u)> predict_state;
  // Jacobian of the state transition function
  // This is a matrix of derivatives of the state prediction function f. Each row i contains the
  // the derivatives df_i / dx_j for every element in the state vector x.
  std::function<F_t(const X& x, K dt, const U& u)> predict_jacobian;
  // State transition noise per time
  // Warning: This must return a symmetric matrix!
  std::function<Q_t(const X& x)> predict_noise;

  // Performs a single prediction step for an extended Kalman filter
  void predict(X& x, P_t& P, K dt, const U& u) const {
    P = ekf_details::EnforceSymmetry(ekf_details::AXAt(P, predict_jacobian(x, dt, u)) +
                                     std::sqrt(dt) * predict_noise(x));
    predict_state(x, dt, u);
  }
  // Performs a single prediction step for an extended Kalman filter without control space
  void predict(X& x, P_t& P, K dt) const {
    static_assert(NU == 0, "Can only omit control vector if control state space has dimension 0");
    return predict(x, P, dt, U{});
  }
};

// Observation model for the Extended Kalman filter
//
// K: type of scalar
// NX: dimension of the state space X (or Eigen::Dynamic for runtime size)
// NZ: dimension of the observation space Z (or Eigen::Dynamic for runtime size)
template <typename X, typename Z>
struct EkfObservationModel {
  using K = typename X::Scalar;
  static_assert(std::is_same<K, typename Z::Scalar>::value,
                "Scalar type for state and observation must be identical");
  static constexpr int NX = X::kDimension;
  static constexpr int NZ = Z::kDimension;
  // Various linear algebra types for state vectors and covariance matrices
  using P_t = EkfCovariance<X>;
  using H_t = EkfObserveJacobian<X, Z>;
  using R_t = EkfCovariance<Z>;

  // State observation function h: X -> Z
  std::function<Z(const X& x)> observe_state;

  // Computes the difference "lhs - rhs" between two observations.
  std::function<Z(const Z& lhs, const Z& rhs)> observe_state_difference;

  // Jacobian of the state observation function
  // This is a matrix of derivatives of the state observation function h. The element J_ij with row
  // row index i and col index j is the derivative dh_i / dx_j.
  std::function<H_t(const X& x)> observe_jacobian;
  // State observation noise
  // Warning: This must return a symmetric matrix!
  std::function<R_t(const X& x, const Z& z)> observe_noise;

  // Performs a single observation step for an extended Kalman filter
  void observe(X& x, P_t& P, const Z& z) const {
    H_t H = observe_jacobian(x);
    Matrix<K, NZ, NZ> S =
        ekf_details::EnforceSymmetry(ekf_details::AXAt(P, H) + observe_noise(x, z));
    // To compute K = P * H^t * S^-1 we solve the linear equation K * S = P * H^t for S.
    // This is identical to solving S^t K^t = H * P^t.
    Matrix<K, NX, NZ> gain =
        S.transpose().colPivHouseholderQr().solve(H * P.transpose()).transpose();
    x.elements += gain * observe_state_difference(z, observe_state(x)).elements;
    const int nx = x.elements.size();  // Need to pass the actual size in case of Eigen::Dynamic
    P = ekf_details::EnforceSymmetry((Matrix<K, NX, NX>::Identity(nx, nx) - gain * H) * P);
  }
};

}  // namespace isaac
