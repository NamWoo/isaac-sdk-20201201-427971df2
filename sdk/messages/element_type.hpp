/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/tensor/element_type.hpp"
#include "messages/element_type.capnp.h"

// Parses element type from proto
isaac::ElementType FromProto(::ElementType proto_element_type);

// Converts element type to corresponding proto
::ElementType ToProto(isaac::ElementType element_type);
