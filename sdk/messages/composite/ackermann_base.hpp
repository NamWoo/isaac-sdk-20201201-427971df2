/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "packages/composite/gems/schema.hpp"

namespace isaac {
namespace messages {

// Schema for a messages::AckermannBaseState. It contains position, speed and linear acceleration
composite::Schema AckermannBaseStateSchema();

// Schema for a messages::AckermannBaseControl. It contains linear acceleration and tire angle.
composite::Schema AckermannBaseControlSchema();

}  // namespace messages
}  // namespace isaac
