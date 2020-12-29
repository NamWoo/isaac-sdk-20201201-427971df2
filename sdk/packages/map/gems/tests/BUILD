"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
cc_test(
    name = "obstacles",
    size = "small",
    srcs = ["obstacles.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core/image",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/gems/image",
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/map/gems:obstacles",
        "@gtest//:main",
    ],
)

