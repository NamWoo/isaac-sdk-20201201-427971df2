"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_component")

isaac_component(
    name = "planner_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        "//packages/planner_cost/gems:planner_cost",
    ],
)

isaac_component(
    name = "addition_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:addition",
    ],
)

isaac_component(
    name = "circles_union_smooth_distance_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner:spherical_robot_shape_component",
        "//packages/planner_cost/gems:circles_union_smooth_distance",
    ],
)

isaac_component(
    name = "range_constraints_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:range_constraints_cost",
    ],
)

isaac_component(
    name = "smooth_minimum_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:smooth_minimum",
    ],
)

isaac_component(
    name = "obstacle_distance_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/map:obstacle_atlas",
        "//packages/planner_cost/gems:obstacle_distance",
    ],
)

isaac_component(
    name = "distance_quadratic_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/map:obstacle_atlas",
        "//packages/planner_cost/gems:distance_quadratic_cost",
    ],
)

isaac_component(
    name = "polyline_distance_quadratic_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:polyline_distance_quadratic_cost",
    ],
)

isaac_component(
    name = "polygon_distance_quadratic_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:polygon_distance_quadratic_cost",
    ],
)

isaac_component(
    name = "polygon_speed_limit_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/map:public_headers",
        "//packages/planner_cost/gems:polygon_speed_limit",
    ],
)

isaac_component(
    name = "dot_product_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:dot_product_cost",
    ],
)

isaac_component(
    name = "smooth_linear_target_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:smooth_linear_target_cost",
    ],
)

isaac_component(
    name = "bounded_quadratic_cost_builder",
    visibility = ["//visibility:public"],
    deps = [
        ":planner_cost_builder",
        "//packages/planner_cost/gems:bounded_quadratic_cost",
    ],
)
