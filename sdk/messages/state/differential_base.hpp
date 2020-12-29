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

// State for a differential base with time (T), position (P), speed (V), and acceleration (A)
template <template <int> class Base>
struct DifferentialBaseTPVALayout : public Base<8> {
  // Time stamp
  ISAAC_COMPOSITE_SCALAR(0, time);
  // Position of the base
  ISAAC_COMPOSITE_SCALAR(1, pos_x);
  ISAAC_COMPOSITE_SCALAR(2, pos_y);
  // Heading of the base
  ISAAC_COMPOSITE_SCALAR(3, heading);
  // Linear speed of the base
  ISAAC_COMPOSITE_SCALAR(4, linear_speed);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(5, angular_speed);
  // Linear acceleration of the base
  ISAAC_COMPOSITE_SCALAR(6, linear_acceleration);
  // Angular acceleration of the base
  ISAAC_COMPOSITE_SCALAR(7, angular_acceleration);
};
using DifferentialBaseTPVAState = DifferentialBaseTPVALayout<state::StateD>;
using DifferentialBaseTPVACompositeConstView =
    composite::TypedCompositeView<const double, DifferentialBaseTPVALayout>;
using DifferentialBaseTPVACompositeView =
    composite::TypedCompositeView<double, DifferentialBaseTPVALayout>;

// State for a differential base with time (T), position (P), and speed (V)
template <template <int> class Base>
struct DifferentialBaseTPVLayout : public Base<6> {
  // Time stamp
  ISAAC_COMPOSITE_SCALAR(0, time);
  // Position of the base
  ISAAC_COMPOSITE_SCALAR(1, pos_x);
  ISAAC_COMPOSITE_SCALAR(2, pos_y);
  // Heading of the base
  ISAAC_COMPOSITE_SCALAR(3, heading);
  // Linear speed of the base
  ISAAC_COMPOSITE_SCALAR(4, linear_speed);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(5, angular_speed);
};
using DifferentialBaseTPVState = DifferentialBaseTPVLayout<state::StateD>;
using DifferentialBaseTPVCompositeConstView =
    composite::TypedCompositeView<const double, DifferentialBaseTPVLayout>;
using DifferentialBaseTPVCompositeView =
    composite::TypedCompositeView<double, DifferentialBaseTPVLayout>;

// State for a differential base with position (P), speed (V), and acceleration (A)
template <template <int> class Base>
struct DifferentialBasePVALayout : public Base<7> {
  // Position of the base
  ISAAC_COMPOSITE_SCALAR(0, pos_x);
  ISAAC_COMPOSITE_SCALAR(1, pos_y);
  // Heading of the base
  ISAAC_COMPOSITE_SCALAR(2, heading);
  // Linear speed of the base
  ISAAC_COMPOSITE_SCALAR(3, linear_speed);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(4, angular_speed);
  // Linear acceleration of the base
  ISAAC_COMPOSITE_SCALAR(5, linear_acceleration);
  // Angular acceleration of the base
  ISAAC_COMPOSITE_SCALAR(6, angular_acceleration);
};
using DifferentialBasePVAState = DifferentialBasePVALayout<state::StateD>;
using DifferentialBasePVACompositeConstView =
    composite::TypedCompositeView<const double, DifferentialBasePVALayout>;
using DifferentialBasePVACompositeView =
    composite::TypedCompositeView<double, DifferentialBasePVALayout>;
// Alias for backward compatibility.
// State vector for a 2D base which can rotate around its origin, but only move in the direction
// of the X axis.
using DifferentialBaseState = DifferentialBasePVAState;

// State for a differential base with time (T), position (P), and speed (V)
template <template <int> class Base>
struct DifferentialBasePVLayout : public Base<5> {
  // Position of the base
  ISAAC_COMPOSITE_SCALAR(0, pos_x);
  ISAAC_COMPOSITE_SCALAR(1, pos_y);
  // Heading of the base
  ISAAC_COMPOSITE_SCALAR(2, heading);
  // Linear speed of the base
  ISAAC_COMPOSITE_SCALAR(3, linear_speed);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(4, angular_speed);
};
using DifferentialBasePVState = DifferentialBasePVLayout<state::StateD>;
using DifferentialBasePVCompositeConstView =
    composite::TypedCompositeView<const double, DifferentialBasePVLayout>;
using DifferentialBasePVCompositeView =
    composite::TypedCompositeView<double, DifferentialBasePVLayout>;

// State for a differential base with speed (V) and acceleration (A)
template <template <int> class Base>
struct DifferentialBaseVALayout : public Base<4> {
  // Linear speed of the base
  ISAAC_COMPOSITE_SCALAR(0, linear_speed);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(1, angular_speed);
  // Linear acceleration of the base
  ISAAC_COMPOSITE_SCALAR(2, linear_acceleration);
  // Angular acceleration of the base
  ISAAC_COMPOSITE_SCALAR(3, angular_acceleration);
};
using DifferentialBaseVAState = DifferentialBaseVALayout<state::StateD>;
// Alias for backward compatibility.
// Observation of the dynamics of the base
using DifferentialBaseDynamics = DifferentialBaseVAState;

// State for a differential base with speed (V)
template <template <int> class Base>
struct DifferentialBaseVLayout : public Base<2> {
  // Linear speed of the base
  ISAAC_COMPOSITE_SCALAR(0, linear_speed);
  // Angular speed of the base
  ISAAC_COMPOSITE_SCALAR(1, angular_speed);
};
using DifferentialBaseVState = DifferentialBaseVLayout<state::StateD>;
using DifferentialBaseVCompositeConstView =
    composite::TypedCompositeView<const double, DifferentialBaseVLayout>;
using DifferentialBaseVCompositeView =
    composite::TypedCompositeView<double, DifferentialBaseVLayout>;
// Alias for backward compatibility.
// Controls used by a differential base
using DifferentialBaseControl = DifferentialBaseVState;

// State for two wheels of a differential base with speed (V) and acceleration (A)
template <template <int> class Base>
struct DifferentialWheelVALayout : public Base<4> {
  // Speeds
  ISAAC_COMPOSITE_SCALAR(0, left_wheel_speed);
  ISAAC_COMPOSITE_SCALAR(1, right_wheel_speed);
  // Accelerations
  ISAAC_COMPOSITE_SCALAR(2, left_wheel_acceleration);
  ISAAC_COMPOSITE_SCALAR(3, right_wheel_acceleration);
};
using DifferentialWheelVAState = DifferentialWheelVALayout<state::StateD>;

}  // namespace messages
}  // namespace isaac
