/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "capnp/serialize.h"
#include "engine/core/buffers/buffer.hpp"

namespace isaac {
namespace serialization {

/*
Cap'n'proto streaming format (all in little-endian)

(4 bytes) The number of segments, minus one (since there is always at least one segment).
(N * 4 bytes) The size of each segment, in words (each word is 8 bytes).
(0 or 4 bytes) Padding up to the next word boundary.
The content of each segment, in order.

from https://capnproto.org/encoding.html
*/

// Decode cap'n'proto flat byte array (represented in a std::string) and 1) extract the segment
// length for each segment stored in segment_lengths, 2) store the content of each segment (body),
// in order, in a vector of uint8_t buffer.
void FlatArrayToCapnpBuffer(CpuBuffer&& bytes, std::vector<size_t>& segment_lengths,
                            std::vector<uint8_t>& buffer);

// Encode cap'n'proto segments into flat byte array (represented in a std::string) according to the
// format described above.
void CapnpSegmentsToFlatArray(const kj::ArrayPtr<const kj::ArrayPtr<const ::capnp::word>> segments,
                              CpuBuffer& bytes);

}  // namespace serialization
}  // namespace isaac
