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

// State to represent any scalar value
template <template <int> class Base>
struct ScalarLayout : public Base<1> {
  ISAAC_COMPOSITE_SCALAR(0, value);
};

using ScalarState = ScalarLayout<state::StateD>;
using ScalarCompositeConstView = composite::TypedCompositeView<const double, ScalarLayout>;
using ScalarCompositeView = composite::TypedCompositeView<double, ScalarLayout>;

}  // namespace messages
}  // namespace isaac
