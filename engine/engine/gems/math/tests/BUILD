"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_test_group")

isaac_cc_test_group(
    size = "small",
    srcs = glob(["*.cpp"]),
    deps = [
        "//engine/gems/math",
        "//engine/gems/math:pose_utils",
        "//engine/gems/math:test_utils",
    ],
)
