/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "capnp/common.h"
#include "engine/core/array/byte_array.hpp"
#include "kj/array.h"

namespace isaac {
namespace serialization {

// Computes the total length of a list of blobs
size_t AccumulateLength(const std::vector<ByteArrayConstView>& blobs);
size_t AccumulateLength(const ByteArrayConstView* pointer, size_t size);

// Copies a list of blobs sequentially to the given output range [dst|dst_end]
byte* CopyAll(const std::vector<ByteArrayConstView>& blobs, byte* dst, byte* dst_end);
byte* CopyAll(const ByteArrayConstView* pointer, size_t size, byte* dst, byte* dst_end);

// Extracts blob lengths into a separate array as 32-bit unsigned integers
void BlobsToLengths32u(const std::vector<ByteArrayConstView>& blobs,
                       std::vector<uint32_t>& lengths);

// Converts a list of cap'n'proto segments into a list of blobs
void CapnpArraysToBlobs(const kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments,
                        std::vector<ByteArrayConstView>& blobs);

}  // namespace serialization
}  // namespace isaac
