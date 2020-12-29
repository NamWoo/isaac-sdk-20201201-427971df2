"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_cc_module")

isaac_cc_module(
    name = "planner_cost",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/planner_cost/components:addition_builder",
        "//packages/planner_cost/components:bounded_quadratic_cost_builder",
        "//packages/planner_cost/components:circles_union_smooth_distance_builder",
        "//packages/planner_cost/components:distance_quadratic_cost_builder",
        "//packages/planner_cost/components:dot_product_cost_builder",
        "//packages/planner_cost/components:obstacle_distance_builder",
        "//packages/planner_cost/components:polygon_distance_quadratic_cost_builder",
        "//packages/planner_cost/components:polygon_speed_limit_builder",
        "//packages/planner_cost/components:polyline_distance_quadratic_cost_builder",
        "//packages/planner_cost/components:range_constraints_cost_builder",
        "//packages/planner_cost/components:smooth_linear_target_cost_builder",
        "//packages/planner_cost/components:smooth_minimum_builder",
    ],
)
