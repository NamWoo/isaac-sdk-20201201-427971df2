/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <vector>

#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// Compute an approximation of the minimum of a list of costs.
// It uses the smooth minimum function below:
//   smooth_min(a1, ... an) = -ln(sum(exp(-alpha * ai)) / alpha
// where alpha is a parameter (called minimum_smoothing below) that controls how close to the
// minimum function it is.
// This smooth minimum function has nice derivative which can be used in gradient descent.
// See https://en.wikipedia.org/wiki/Smooth_maximum for more information
class SmoothMinimum : public PlannerCost {
 public:
  SmoothMinimum() = default;

  // This PlannerCost considers a state as valid if each cost function that depends on it consider
  // it as valid.
  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Adds a PlannerCost to the list.
  void addCost(PlannerCost* cost) {
    if (cost == nullptr) return;
    costs_.push_back(cost);
    values_.push_back(0.0);
  }

  // Sets the smoothing factor.
  void setMinimumSmoothing(double minimum_smoothing) {
    ASSERT(!IsAlmostZero(minimum_smoothing), "minimum_smoothing should not be zero");
    minimum_smoothing_ = minimum_smoothing;
  }

 private:
  // Store the list of PlannerCost to be evaluated. The smooth minimum of them will be returned.
  std::vector<PlannerCost*> costs_;
  // Used to store the list of evaluation of each cost function above. Storing it as global prevent
  // constant memory allocation.
  std::vector<double> values_;
  // Smoothing parameter. If negative this function will actually compute the smooth maximum.
  // This value should never be 0
  double minimum_smoothing_ = 20.0;
};

}  // namespace planner_cost
}  // namespace isaac
