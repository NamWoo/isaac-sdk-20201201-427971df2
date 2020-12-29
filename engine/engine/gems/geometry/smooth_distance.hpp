/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cmath>
#include <functional>

#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"
#include "engine/gems/geometry/polyline.hpp"

namespace isaac {
namespace geometry {

// Returns the smooth distance to a polyline. This is an implementation of the Signed Lp distance
// fields described in this paper: http://home.eps.hw.ac.uk/~ab226/papers/dist.pdf
// Here were are computing the distance psi_1(x) = 1 / phi_1(x).
// Section 4 in the paper provides the result of the integral for a single segment, we will be
// iterating over all the segments of the polyline.
// An additional simplifaction will be made:
// The formula: phi_1(x) = sum((1/a) + 1/b)) * tan(|gamma|/2) requires computing tan(gamma/2).
// We can use the formula
//   tan(gamma|/2) = sin(gamma) / (1 + cos(gamma)) == a.cross(b) / (ab + a.dot(b))
// This allows to simplify the computation as we don't need to compute gamma, and afterwards take
// the derivative to compute the gradient.
// Note:
//   - This returns a signed distance with the convention that positive means the point lays on
//     the left side of poyline.
//   - If the polyline is empty, a default distance of 0 is returned and the gradient is null.
//   - If the polyline contains a single point, the distance to this point is returned. This is not
//     a smooth distance due to the singularity at that point.
template<typename K>
K SmoothDistanceToPolyline(const Polyline<K, 2>& polyline, const Vector2<K>& p,
                           Vector2<K>* gradient = nullptr) {
  // Handle edge cases.
  if (polyline.size() <= 2) {
    if (polyline.empty()) {
      if (gradient) gradient->setZero();
      return K(0);
    }
    const Vector2<K> delta = p - polyline.front();
    if (polyline.size() == 2) {
      const Vector2<K> ray = (polyline.back() - polyline.front()).normalized();
      const Vector2<K> ray_normal(-ray[1], ray[0]);
      if (gradient) *gradient = ray_normal;
      return delta.dot(ray_normal);
    }
    const K distance = delta.norm();
    if (gradient) {
      if (IsAlmostZero(distance)) {
        gradient->setZero();
      } else {
        *gradient = delta.normalized();
      }
    }
    return distance;
  }
  K phi_1 = K(0);
  if (gradient) gradient->setZero();

  // Helper structure to cache information needed along the computation to avoid recomputing it too
  // many times.
  struct HelperStruct {
    // If ray is set to true, it means it represent an infinite ray, the norm will be computed
    // normally because it's needed to normalize the dot/cross product, however the inverse norm
    // will be used normally and therefore set to 0.
    HelperStruct(const Vector2<K>& delta, bool ray) {
      this->delta = delta;
      norm = delta.norm();
      if (ray) {
        norm_inv = K(0);
      } else {
        // If norm is 0, norm_inv will never be used.
        norm_inv = K(1) / norm;
      }
    }
    // Delta vector between p and a point on the polyline
    Vector2<K> delta;
    // Norm of delta
    K norm;
    // Inverse of norm of delta
    K norm_inv;
  };

  // Helper function to update the gradient.
  // It takes both helper structure as well as the derivative of the cross product, dot product, and
  // product of the norm which all appear in the computation of the tangent. This will allow to
  // compute the gradient relative to the vector `a`.
  // Distance = (1/|a| + 1/|b|) * (|a|*|b| - a * b) / (a x b)
  // There are 3 vector components:
  //  - One in the direction of `a` coming from diff(|a|) = a / |a|
  //  - One in the direction of `b` coming from a * b (the dot product)
  //  - One in the direction orthogonal to `b` coming from the cross product a x b.
  auto updateGradient = [&](const HelperStruct& a, const HelperStruct& b, K dcross, K ddot, K dprod,
                            K tan) {
    if (gradient == nullptr || a.norm_inv == K(0)) return;
    *gradient += dcross * Vector2<K>(b.delta[1], -b.delta[0])
              + (dprod * b.norm - tan * a.norm_inv * a.norm_inv) * a.norm_inv * a.delta
              + ddot * b.delta;
  };

  const bool is_polygon = IsAlmostZero((polyline.front() - polyline.back()).squaredNorm());
  HelperStruct current_data = is_polygon ? HelperStruct(polyline[0] - p, false)
                                         : HelperStruct(polyline[0] - polyline[1], true);
  for (size_t idx = 1; idx < polyline.size(); idx++) {
    const HelperStruct next_data =
        idx+1 != polyline.size() || is_polygon
            ? HelperStruct(polyline[idx] - p, false)
            : HelperStruct(polyline[idx] - polyline[idx - 1], true);

    const K dot_product = current_data.delta.dot(next_data.delta);
    const K cross_product = CrossProduct(current_data.delta, next_data.delta);
    const K norm_product = current_data.norm * next_data.norm;

    // If the cross product is null && dot product < 0, it means we are on a segement and the
    // distance is exactly 0
    const K prod_plus_dot = norm_product + dot_product;
    if (IsAlmostZero(prod_plus_dot)) {
      if (gradient) {
        *gradient = Vector2<K>(current_data.delta.y(), -current_data.delta.x()).normalized();
      }
      return K(0);
    }
    const K sum_inv = current_data.norm_inv + next_data.norm_inv;
    const K prod_plus_dot_inv = K(1) / prod_plus_dot;
    const K dcross = sum_inv * prod_plus_dot_inv;
    const K tan = cross_product * prod_plus_dot_inv;
    const K update = tan * sum_inv;
    const K dprod = -update * prod_plus_dot_inv;
    const K ddot = dprod;
    phi_1 += update;
    // Update the gradient relative to the current point.
    updateGradient(current_data, next_data, dcross, ddot, dprod, tan);
    // Update the gradient relative to the next point.
    updateGradient(next_data, current_data, -dcross, ddot, dprod, tan);

    current_data = next_data;
  }
  // psi_p(x) = (1/phi_p(x))^(1/p)
  // Here we use p == 1 to simplify, however the paper also suggest a normalization by 2^(1/p) to
  // better approximate the distance.
  if (gradient) *gradient /= K(0.5) * (phi_1 * phi_1);
  return K(2) / phi_1;
}

}  // namespace geometry
}  // namespace isaac
