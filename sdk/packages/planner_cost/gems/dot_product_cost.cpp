/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "packages/planner_cost/gems/dot_product_cost.hpp"

namespace isaac {
namespace planner_cost {

double DotProductCost::evaluate(double /*time*/, const VectorXd& state) {
  ASSERT(dot_vector_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), dot_vector_.size());
  return offset_ + dot_vector_.dot(state);
}

void DotProductCost::addGradient(double /*time*/, const VectorXd& state,
                                 Eigen::Ref<VectorXd> gradient) {
  ASSERT(dot_vector_.size() == state.size(), "Dimension mismatch: %d vs %d",
         state.size(), dot_vector_.size());
  gradient += dot_vector_;
}

}  // namespace planner_cost
}  // namespace isaac
