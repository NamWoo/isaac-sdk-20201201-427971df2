/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

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

// State for a holonomic base with position (P), speed (V), and acceleration (A)
template <template <int> class Base>
struct HolonomicBasePVALayout : public Base<8> {
  // Position of the base
  ISAAC_COMPOSITE_SCALAR(0, pos_x);
  ISAAC_COMPOSITE_SCALAR(1, pos_y);
  // Heading of the base
  ISAAC_COMPOSITE_SCALAR(2, heading);
  // Linear velocity of the base
  ISAAC_COMPOSITE_SCALAR(3, speed_x);
  ISAAC_COMPOSITE_SCALAR(4, speed_y);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(5, angular_speed);
  // Linear acceleration of the base
  ISAAC_COMPOSITE_SCALAR(6, acceleration_x);
  ISAAC_COMPOSITE_SCALAR(7, acceleration_y);
};
using HolonomicBasePVAState = HolonomicBasePVALayout<state::StateD>;
// Alias for backward compatibility.
// State vector for a 2D holonomic base.
using HolonomicBaseState = HolonomicBasePVAState;

// State for a holonomic base with position (P) and speed (V)
template <template <int> class Base>
struct HolonomicBasePVLayout : public Base<5> {
  // Linear velocity of the base
  ISAAC_COMPOSITE_SCALAR(0, speed_x);
  ISAAC_COMPOSITE_SCALAR(1, speed_y);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(2, angular_speed);
  // Linear acceleration of the base
  ISAAC_COMPOSITE_SCALAR(3, acceleration_x);
  ISAAC_COMPOSITE_SCALAR(4, acceleration_y);
};
using HolonomicBasePVState = HolonomicBasePVLayout<state::StateD>;
// Alias for backward compatibility.
// Observation of the dynamics of the holonomic base
using HolonomicBaseDynamics = HolonomicBasePVState;

// State for a holonomic base with speed (V)
template <template <int> class Base>
struct HolonomicBaseVLayout : public Base<3> {
  // Linear velocity of the base
  ISAAC_COMPOSITE_SCALAR(0, speed_x);
  ISAAC_COMPOSITE_SCALAR(1, speed_y);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(2, angular_speed);
};
using HolonomicBaseVState = HolonomicBaseVLayout<state::StateD>;
// Alias for backward compatibility.
// Controls used by a holonomic base
using HolonomicBaseControls = HolonomicBaseVState;

}  // namespace messages
}  // namespace isaac
