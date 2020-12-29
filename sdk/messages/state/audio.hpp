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

// Angle of the dominant sound source in radians
template <template <int> class Base>
struct SourceAngleLayout : public Base<1> {
  // Counter-clockwise angle in radians
  ISAAC_COMPOSITE_SCALAR(0, angle);
};
using SourceAngleState = SourceAngleLayout<state::StateD>;

// Average energy of an audio packet in decibels (dB)
template <template <int> class Base>
struct SourceEnergyLayout : public Base<1> {
  // Energy in decibels (dB)
  ISAAC_COMPOSITE_SCALAR(0, energy);
};
using AudioEnergyState = SourceEnergyLayout<state::StateD>;

}  // namespace messages
}  // namespace isaac
