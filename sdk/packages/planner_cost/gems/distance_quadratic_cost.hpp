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

#include "engine/core/math/types.hpp"
#include "packages/planner_cost/gems/planner_cost.hpp"

namespace isaac {
namespace planner_cost {

// Computes a quadratic cost function based on a distance function and the speed:
// Several cost function of the following form will be added together:
//   0.5 * gain * std::min(0, distance_function_ - target_distance - speed_gradient * |speed|)^2
// In order to be differentiable, the |speed| is replaced with sqrt(epsilon + speed^2).
// The distance_function used here needs to expect the state to be Vector3d representing:
//  [pos_x, pos_y, heading]
class DistanceQuadraticCost : public PlannerCost {
 public:
  DistanceQuadraticCost() = default;

  bool isValid(double time, const VectorXd& state) override;
  double evaluate(double time, const VectorXd& state) override;
  void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override;
  void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override;

  // Sets a PlannerCost to the list.
  void setDistanceFunction(PlannerCost* distance_function) {
    distance_function_ = distance_function;
  }

  // Sets the indices of the [pos_x, pos_y, heading, linear_speed].
  void setIndices(const Vector4i& indices) {
    indices_ = indices;
  }

  // Adds a new cost:
  //  0.5 * gain * std::min(0, distance_function_ - target_distance - speed_gradient * speed)^2
  void addCostFunction(double gain, double target_distance, double speed_gradient = 0.0) {
    costs_.push_back({gain, target_distance, speed_gradient});
  }

  // Removes all the cost functions
  void resetCostFunctions() {
    costs_.clear();
  }

  // Controls whether the hessian of each obstacle will be assumed to be zero or computed.
  // The distance function is unfortunately not differentiable, we are approximating it in order
  // to keep it differentiable. One down side is, the better the approximation the more ill
  // conditionned are the hessian. This can produce issues therefore this flag allow using a zero
  // hessian (which is the correct hessian of the distance to a line, and a good approximation of
  // the hessian of the distance from a point when the point is far enough).
  void setUseZeroHessian(bool use_zero_hessian) {
    use_zero_hessian_ = use_zero_hessian;
  }

 private:
  // Parameters holder for a cost associated to a distance:
  // The cost will be: gain * min(0, distance - speed_gradient * |speed| - target_distance)^2
  struct CostParameters {
    // Gain associated to the cost
    double gain;
    // Target distance when the speed is 0
    double target_distance;
    // How much the target distance increases when the |speed| increase.
    double speed_gradient;
  };

  // Indices of pos_x, pos_y, heading, and linear_speed.
  Vector4i indices_ = Vector4i(0, 1, 2, 3);
  PlannerCost* distance_function_ = nullptr;
  std::vector<CostParameters> costs_;
  bool use_zero_hessian_ = true;
};

}  // namespace planner_cost
}  // namespace isaac
