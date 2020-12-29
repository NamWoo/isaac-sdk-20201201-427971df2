"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "kinematic",
    srcs = ["kinematic.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/math/gems/kinematic_tree",
        "//packages/math/gems/kinematic_tree:kinematic",
        "//packages/math/gems/kinematic_tree:motors",
        "@gtest//:main",
    ],
)

cc_test(
    name = "json_loader",
    srcs = ["json_loader.cpp"],
    data = glob(["*.kinematic.json"]),
    deps = [
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/math/gems/kinematic_tree",
        "//packages/math/gems/kinematic_tree:json_loader",
        "//packages/math/gems/kinematic_tree:kinematic",
        "//packages/math/gems/kinematic_tree:motors",
        "@gtest//:main",
    ],
)
