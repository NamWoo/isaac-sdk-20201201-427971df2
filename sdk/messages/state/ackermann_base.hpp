/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "packages/composite/gems/typed_composite_view.hpp"
#include "packages/engine_gems/state/state.hpp"

namespace isaac {
namespace messages {

// State for an ackermann base wich contains the position of the base, the speed, and the linear
// acceleration.
template <template <int> class Base>
struct AckermannBaseStateLayout : public Base<6> {
  // Position of the base
  ISAAC_COMPOSITE_SCALAR(0, pos_x);
  ISAAC_COMPOSITE_SCALAR(1, pos_y);
  // Heading of the base
  ISAAC_COMPOSITE_SCALAR(2, heading);
  // Tire angle (0 means going straight).
  ISAAC_COMPOSITE_SCALAR(3, tire_angle);
  // Linear velocity of the base
  ISAAC_COMPOSITE_SCALAR(4, linear_speed);
  // Rate of change of the tire angle
  ISAAC_COMPOSITE_SCALAR(5, linear_acceleration);
};
using AckermannBaseState = AckermannBaseStateLayout<state::StateD>;

// Control for an ackermann base using the linear acceleration and the tire angle.
template <template <int> class Base>
struct AckermannBaseControlLayout : public Base<2> {
  // Linear acceleration of the base
  ISAAC_COMPOSITE_SCALAR(0, linear_acceleration);
  // Rate of change of the tire angle
  ISAAC_COMPOSITE_SCALAR(1, tire_angle);
};
using AckermannBaseControl = AckermannBaseControlLayout<state::StateD>;

}  // namespace messages
}  // namespace isaac
