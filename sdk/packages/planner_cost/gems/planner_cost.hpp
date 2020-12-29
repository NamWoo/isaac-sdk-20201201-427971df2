/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/assert.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace planner_cost {

// Interface for a planner cost function.
// This interface can be used to generate a cost based on a given state. This can be used by some
// planner algorithm such as the LQR. Some of those algorithms require the function addCostHessian
// to be implemented, therefore the cost function needs to be D2 (twice differentiable).
class PlannerCost {
 public:
  // Virtual default destructor
  virtual ~PlannerCost() = default;

  // Returns true if the current state is valid.
  virtual bool isValid(double time, const VectorXd& state) { return true; }

  // Returns the evaluation at a given state and time.
  virtual double evaluate(double time, const VectorXd& state) = 0;

  // Adds the gradient of this cost function for the given state to the given vector `gradient`.
  // `gradient` uses a Ref<VectorXd> to allow block operation to passed to this function.
  virtual void addGradient(double time, const VectorXd& state,
                           Eigen::Ref<Eigen::VectorXd> gradient) = 0;

  // Adds the hessian of this cost function for the given state to the given matrix `hessian`.
  // `hessian` uses a Ref<VectorXd> to allow block operation to passed to this function.
  virtual void addHessian(double time, const VectorXd& state,
                          Eigen::Ref<Eigen::MatrixXd> hessian) = 0;
};

}  // namespace planner_cost
}  // namespace isaac
