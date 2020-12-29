/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

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

// State for a pan/tilt system with position (P) and speed (V)
template <template <int> class Base>
struct PanTiltPVLayout : public Base<4> {
  ISAAC_COMPOSITE_SCALAR(0, pan);
  ISAAC_COMPOSITE_SCALAR(1, tilt);
  ISAAC_COMPOSITE_SCALAR(2, pan_speed);
  ISAAC_COMPOSITE_SCALAR(3, tilt_speed);
};
using PanTiltPVState = PanTiltPVLayout<state::StateD>;
// Alias for backward compatibility.
// State of a pan/tilt unit which can rotate around two axes
using PanTiltState = PanTiltPVState;

}  // namespace messages
}  // namespace isaac
