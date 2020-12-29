"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "bipartite_graph_matching",
    srcs = ["bipartite_graph_matching.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core"],
)

isaac_cc_library(
    name = "bresenham",
    srcs = ["bresenham.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core/math"],
)

isaac_cc_library(
    name = "flow_control",
    srcs = ["flow_control.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "more_type_traits",
    srcs = ["more_type_traits.hpp"],
    visibility = ["//visibility:public"],
)

isaac_cc_library(
    name = "string_utils",
    srcs = ["string_utils.hpp"],
    visibility = ["//visibility:public"],
)

isaac_cc_library(
    name = "timeseries",
    srcs = ["timeseries.hpp"],
    visibility = ["//visibility:public"],
    deps = ["//engine/core"],
)

isaac_cc_library(
    name = "algorithm",
    visibility = ["//visibility:public"],
    deps = [
        ":bipartite_graph_matching",
        ":bresenham",
        ":flow_control",
        ":more_type_traits",
        ":string_utils",
        ":timeseries",
    ],
)
