/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "sop_point_cloud.hpp"

#include <vector>

#include "engine/gems/serialization/base64.hpp"

namespace isaac {
namespace sight {

namespace {

// Create point cloud with 2d or 3d coordinates. N - dimension
template <int N>
void CreateSopPointCloud(SampleCloudConstView<float, N>  points,
                         SampleCloudConstView3f colors,
                         int downsample_stride, Json& json) {
  json["t"] = "point_cloud";
  json["dim"] = N;
  if (downsample_stride == 1) {
    json["points"] = serialization::Base64Encode(
        reinterpret_cast<const uint8_t*>(points.data().begin()), points.data().size());
    json["colors"] = serialization::Base64Encode(
        reinterpret_cast<const uint8_t*>(colors.data().begin()), colors.data().size());
  } else {
    SampleCloud<float, N> downsample_points;
    const int point_count = points.dimensions()[0] / downsample_stride;
    if (point_count > 0) {
      downsample_points.resize(point_count);
    }
    SampleCloud3f downsample_colors;
    const int color_count = colors.dimensions()[0] / downsample_stride;
    if (color_count > 0) {
      downsample_colors.resize(color_count);
    }
    for (int index = 0; index < point_count; index++) {
      downsample_points(index) = points(index * downsample_stride);
    }
    for (int index = 0; index < color_count; index++) {
      downsample_colors(index) = colors(index * downsample_stride);
    }
    json["points"] = serialization::Base64Encode(
        reinterpret_cast<const uint8_t*>(downsample_points.data().begin()),
        downsample_points.data().size());
    json["colors"] = serialization::Base64Encode(
        reinterpret_cast<const uint8_t*>(downsample_colors.data().begin()),
        downsample_colors.data().size());
  }
}

}  // namespace

SopPointCloud SopPointCloud::Create(SampleCloudConstView3f  points, SampleCloudConstView3f colors,
                                    int downsample_stride) {
  SopPointCloud sop;
  CreateSopPointCloud<3>(points, colors, downsample_stride, sop.json_);
  return sop;
}

SopPointCloud SopPointCloud::Create(SampleCloudConstView2f  points, SampleCloudConstView3f colors,
                                    int downsample_stride) {
  SopPointCloud sop;
  CreateSopPointCloud<2>(points, colors, downsample_stride, sop.json_);
  return sop;
}

}  // namespace sight
}  // namespace isaac
