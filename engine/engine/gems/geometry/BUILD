"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_library")

cc_library(
    name = "geometry",
    visibility = ["//visibility:public"],
    deps = [
        ":line_utils",
        ":pinhole",
        ":plane_utils",
        ":transform_to_plane",
        ":types",
    ],
)

isaac_cc_library(
    name = "line",
    hdrs = ["line.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "line_segment",
    hdrs = ["line_segment.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "polyline",
    hdrs = ["polyline.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "polygon",
    hdrs = ["polygon.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":line_segment",
        ":line_utils",
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "n_cuboid",
    hdrs = ["n_cuboid.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "n_sphere",
    hdrs = ["n_sphere.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "pinhole",
    hdrs = ["pinhole.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "plane",
    hdrs = ["plane.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "plane_utils",
    hdrs = ["plane_utils.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":plane",
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "transform_to_plane",
    hdrs = ["transform_to_plane.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":pinhole",
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "types",
    hdrs = ["types.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":line",
        ":line_segment",
        ":n_cuboid",
        ":n_sphere",
        ":plane",
        ":polygon",
        ":polyline",
    ],
)

isaac_cc_library(
    name = "line_utils",
    hdrs = ["line_utils.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":polyline",
        "//engine/core",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "convex_hull",
    hdrs = ["convex_hull.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":polygon",
        "//engine/core/math",
    ],
)

isaac_cc_library(
    name = "smooth_distance",
    hdrs = ["smooth_distance.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":polyline",
        "//engine/core",
        "//engine/core/math",
    ],
)
