"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "planner_cost",
    hdrs = ["planner_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "range_constraints_cost",
    srcs = ["range_constraints_cost.cpp"],
    hdrs = ["range_constraints_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "smooth_linear_target_cost",
    srcs = ["smooth_linear_target_cost.cpp"],
    hdrs = ["smooth_linear_target_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "pose2_transform",
    srcs = ["pose2_transform.cpp"],
    hdrs = ["pose2_transform.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "addition",
    srcs = ["addition.cpp"],
    hdrs = ["addition.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "scalar_multiplication",
    srcs = ["scalar_multiplication.cpp"],
    hdrs = ["scalar_multiplication.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "scalar_addition",
    srcs = ["scalar_addition.cpp"],
    hdrs = ["scalar_addition.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "smooth_minimum",
    srcs = ["smooth_minimum.cpp"],
    hdrs = ["smooth_minimum.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "circles_union_smooth_distance",
    srcs = ["circles_union_smooth_distance.cpp"],
    hdrs = ["circles_union_smooth_distance.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        ":pose2_transform",
        ":scalar_addition",
        ":smooth_minimum",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/geometry:n_sphere",
    ],
)

isaac_cc_library(
    name = "obstacle_distance",
    srcs = ["obstacle_distance.cpp"],
    hdrs = ["obstacle_distance.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "//packages/map/gems:obstacles",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "distance_quadratic_cost",
    srcs = ["distance_quadratic_cost.cpp"],
    hdrs = ["distance_quadratic_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "//packages/map/gems:obstacles",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "polyline_distance_quadratic_cost",
    srcs = ["polyline_distance_quadratic_cost.cpp"],
    hdrs = ["polyline_distance_quadratic_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "//packages/map/gems:obstacles",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/geometry:polyline",
        "@com_nvidia_isaac_engine//engine/gems/geometry:smooth_distance",
    ],
)

isaac_cc_library(
    name = "polygon_distance_quadratic_cost",
    srcs = ["polygon_distance_quadratic_cost.cpp"],
    hdrs = ["polygon_distance_quadratic_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "//packages/map/gems:obstacles",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/geometry:polygon",
        "@com_nvidia_isaac_engine//engine/gems/geometry:polyline",
        "@com_nvidia_isaac_engine//engine/gems/geometry:smooth_distance",
    ],
)

isaac_cc_library(
    name = "polygon_speed_limit",
    srcs = ["polygon_speed_limit.cpp"],
    hdrs = ["polygon_speed_limit.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/geometry:polygon",
        "@com_nvidia_isaac_engine//engine/gems/geometry:polyline",
        "@com_nvidia_isaac_engine//engine/gems/geometry:smooth_distance",
    ],
)

isaac_cc_library(
    name = "dot_product_cost",
    srcs = ["dot_product_cost.cpp"],
    hdrs = ["dot_product_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "bounded_quadratic_cost",
    srcs = ["bounded_quadratic_cost.cpp"],
    hdrs = ["bounded_quadratic_cost.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)
