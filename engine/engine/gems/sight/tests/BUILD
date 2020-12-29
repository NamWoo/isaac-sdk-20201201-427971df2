"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_binary")

isaac_cc_binary(
    name = "benchmark",
    srcs = ["benchmark.cpp"],
    deps = [
        "//engine/gems/sight",
        "@benchmark",
    ],
)

cc_test(
    name = "json_key_truncation",
    srcs = ["json_key_truncation.cpp"],
    data = glob(["test_data/*.json"]),
    deps = [
        "//engine/gems/serialization:json",
        "//engine/gems/sight",
        "@gtest//:main",
    ],
)

cc_test(
    name = "sop_from_file",
    srcs = ["sop_from_file.cpp"],
    data = glob(["test_data/*.json"]),
    deps = [
        "//engine/gems/serialization:json",
        "//engine/gems/sight",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kinematic_json_test",
    srcs = ["kinematic_json_test.cpp"],
    data = glob(["test_data/*.json"]),
    deps = [
        "//engine/gems/sight:kinematics_json",
        "@gtest//:main",
    ],
)
