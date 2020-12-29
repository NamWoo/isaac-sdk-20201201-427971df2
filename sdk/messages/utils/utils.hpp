/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/alice/message.hpp"
#include "engine/core/buffers/buffer.hpp"

namespace isaac {
namespace utils {
// Creates Isaac Message from data and buffers
alice::MessageBasePtr CreateProtoMessage(const Uuid uuid_str, const uint64_t type_id,
                                         CpuBuffer&& proto_bytes, std::vector<CpuBuffer>&& buffers);

alice::MessageBasePtr CreateProtoMessage(const Uuid uuid, const uint64_t type_id,
                                         std::vector<CpuBuffer>&& segments,
                                         std::vector<CpuBuffer>&& buffers);
}  // namespace utils
}  // namespace isaac
