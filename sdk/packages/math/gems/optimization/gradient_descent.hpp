/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <utility>

namespace isaac {
namespace optimization {

// An iterative line search algorithm based on backtracking using the Armijo-Golstein condition.
// The step length `step` is initialized with the given value `step` and multiplied by `tau` until
// the decrease in score reaches a value expected by the given `slope` computed based on the
// gradient of the function. The function f: R -> R, f(a) = f(x + a * p) is evaluated to compute the
// value for a given step length. As the function is likely already evaluated at 0 before starting
// the line search the value f(0) is given as a function parameter `f0`. The Armijo-Goldstein
// condition is defined by the control factor `armijo_goldstein_control`. The function `f` will
// return true if the Armijo-Golstein condition was true before the maximum number of iterations are
// reached.
template <typename F>
bool BacktrackLineSearch(F&& f, double f0, double slope, double step, double tau,
                         double armijo_goldstein_control, unsigned max_iterations) {
  const double t = - armijo_goldstein_control * slope;
  for (unsigned i = 0; i < max_iterations; i++) {
    const double delta = f0 - f(step);
    if (delta > step * t) {
      return true;
    }
    step *= tau;
  }
  return false;
}

// Parameters for the gradient descent algorithm
struct GradientDescentParameters {
  // The maxium number of iterations. Each iteration will do a line search.
  unsigned max_iterations = 25;
  // If the norm of the gradient falls below this threshold convergence is achieved.
  double gradient_norm_target = 0.001;
  // Multiplicative factor on step length during line search for every failed iteration
  double line_search_step_factor = 0.5;
  // Armijo-Goldstein parameter for line search
  double line_search_armijo_goldstein_control = 0.5;
  // Maximum number of iterations for line search
  unsigned line_search_max_iterations = 25;
  // If this is set to true a line search convergence failure will result in overall failure
  bool break_on_failed_line_search = false;
};

// A type which holds various pieces of information from the line search algorithm
struct GradientDescentInfo {
  // The score for the state as computed by the given value function
  double score;
  // The norm of the last gradient used during line search
  double gradient_norm;
  // The number of iterations taken by algorithm. This counts the number of times line serach ran.
  unsigned num_iterations;
  // Wether the algorithm converged. This is true if the gradient norm was sufficiently small
  // before any of the exit conditions became true.
  bool converged;
};

// A gradient descent algorithm using backtracking line search which minimizes a given function.
// The function is given in two variants `value_f` defined as `value_f: State -> Scalar` and
// as `value_and_gradient_f` defined as `value_and_gradient_f: State -> Scalar x Tangent`. Here
// `State` is the domain over which the function is to be minimized and `Tangent` is the
// corresponding tangent space. `Scalar` must be double for the current implementation for
// simplicity. Two separate functions are used to allow optimizations for function evaluation during
// line search when the gradient is not necessary. The function `update_f` defined as
// `update_f: State x Tangent -> State` defines the exponential map on the state space. Various
// parameters to control the behavior of the gradient descent algorithm can be passed via the
// given `parameters`. As this is a local optimization algorithm a start state is required and
// given via `state_0`. The function returns a pair with the state it converged to and some other
// bits of useful information
template <typename State, typename Tangent = State,
          typename ValueF, typename ValueAndGradientF,
          typename UpdateF>
std::pair<State, GradientDescentInfo> GradientDescent(
    State state, ValueF&& value_f, ValueAndGradientF&& value_and_gradient_f,
    UpdateF&& update_f, GradientDescentParameters parameters = GradientDescentParameters{}) {
  GradientDescentInfo info;
  info.score = 0.0;
  info.gradient_norm = 0.0;
  info.converged = false;
  info.num_iterations = 0;

  while (true) {
    // Update and check iteration count
    if (info.num_iterations >= parameters.max_iterations) {
      break;
    }
    info.num_iterations++;

    // Compute value and gradient for current position
    const std::pair<double, Tangent> value_and_gradient = value_and_gradient_f(state);

    // Update and check score and norm of gradient
    info.score = value_and_gradient.first;
    info.gradient_norm = value_and_gradient.second.norm();
    if (info.gradient_norm < parameters.gradient_norm_target) {
      info.converged = true;
      break;
    }

    // Search into the opposite direction than the gradient as we are minimizing
    const Tangent search_direction = value_and_gradient.second / (-info.gradient_norm);
    const double slope = -info.gradient_norm;

    // Run a back-tracking line search. State and score will be updated continuously while running
    // the line search algorithm.
    const State origin = state;
    double step = 1.0 + info.gradient_norm;  // FIXME: What is a good starting step size?
    const bool ok = BacktrackLineSearch(
        [&] (double step) {
          state = update_f(origin, step * search_direction);
          info.score = value_f(state);
          return info.score;
        },
        info.score, slope, step, parameters.line_search_step_factor,
        parameters.line_search_armijo_goldstein_control, parameters.line_search_max_iterations);
    if (!ok && parameters.break_on_failed_line_search) {
      break;
    }
  }
  return {std::move(state), info};
}

}  // namespace optimization
}  // namespace isaac
