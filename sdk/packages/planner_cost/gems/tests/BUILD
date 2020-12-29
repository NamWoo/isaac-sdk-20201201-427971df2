"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "range_constraints_cost",
    srcs = ["range_constraints_cost.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/planner_cost/gems:range_constraints_cost",
        "@gtest//:main",
    ],
)

cc_test(
    name = "addition",
    srcs = ["addition.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/planner_cost/gems:addition",
        "//packages/planner_cost/gems:range_constraints_cost",
        "@gtest//:main",
    ],
)

cc_test(
    name = "smooth_minimum",
    srcs = ["smooth_minimum.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/planner_cost/gems:range_constraints_cost",
        "//packages/planner_cost/gems:smooth_minimum",
        "@gtest//:main",
    ],
)

cc_test(
    name = "pose2_transform",
    srcs = ["pose2_transform.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/math:pose_utils",
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/planner_cost/gems:pose2_transform",
        "//packages/planner_cost/gems:range_constraints_cost",
        "@gtest//:main",
    ],
)
