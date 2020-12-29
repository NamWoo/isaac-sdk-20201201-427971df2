/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// Compute the dot product between a state and a constant vector.
// It returns offset_ + dot_vector_.dot(state)
class DotProductCost : public PlannerCost {
 public:
  DotProductCost() = default;

  // Returns offset_ + state.dot(dot_vector_);
  double evaluate(double time, const VectorXd& state) override;
  // The gradient is simply dot_vector_.
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  // Hessian is simple, it's zero.
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override {}

  // Sets the vector to use in the dot product multiplication.
  void setDotVector(const VectorXd& dot_vector) {
    ASSERT(dot_vector_.size() == 0 || dot_vector_.size() == dot_vector.size(),
           "The expected dimension of state cannot be changed from %d to %d",
           dot_vector_.size(), dot_vector.size());
    dot_vector_ = dot_vector;
  }

  // Sets the offset added to the dot product.
  void setOffset(double offset) {
    offset_ = offset;
  }

  // Accessor to the current dot_vector
  VectorXd& dot_vector() { return dot_vector_; }
  const VectorXd& dot_vector() const { return dot_vector_; }

 private:
  VectorXd dot_vector_;
  double offset_ = 0.0;
};

}  // namespace planner_cost
}  // namespace isaac
