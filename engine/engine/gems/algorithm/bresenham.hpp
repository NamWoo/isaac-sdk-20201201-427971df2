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

#include "engine/core/math/types.hpp"

namespace isaac {

namespace bresenham_details {

// Implementation for Bresenham
template<bool FlipXY, typename F>
bool BresenhamImpl(int u0, int v0, int u1, int v1, F f) {
  int du = u1 - u0;
  int ui = 1;
  if (du < 0) {
    du = -du;
    ui = -1;
  }
  int dv = v1 - v0;
  int vi = 1;
  if (dv < 0) {
    dv = -dv;
    vi = -1;
  }
  int D = 2*dv - du;
  int v = v0;
  for (int u=u0; ; u+=ui) {
    const bool ok = (FlipXY ? f(v, u) : f(u, v));
    if (!ok) {
      return false;
    }
    if (D > 0) {
      v += vi;
      D -= 2*du;
    }
    D += 2*dv;
    if (u == u1) {
      break;
    }
  }
  return true;
}

}  // namespace bresenham_details

// Traces a path from start to end through a 2D grid. For each pixel on the traced path the given
// function f is called with the current pixel coordinates. If the function returns false the trace
// will be stopped early, otherwise it will continue until the end pixel is reached.
template<typename F>
bool Bresenham(const Vector2i& start, const Vector2i& end, F f) {
  const int x0 = start[0];
  const int y0 = start[1];
  const int x1 = end[0];
  const int y1 = end[1];
  if (std::abs(y1 - y0) < std::abs(x1 - x0)) {
    return bresenham_details::BresenhamImpl<false>(x0, y0, x1, y1, std::move(f));
  } else {
    return bresenham_details::BresenhamImpl<true>(y0, x0, y1, x1, std::move(f));
  }
}

}  // namespace isaac
