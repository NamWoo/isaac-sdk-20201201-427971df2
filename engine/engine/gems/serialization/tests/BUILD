"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:cc_capnp_library.bzl", "cc_capnp_library")

cc_test(
    name = "tests",
    srcs = glob(["*.cpp"]),
    data = [
        ":test_data/sample_a.json",
        ":test_data/sample_b.json",
        ":test_data/sample_c.json",
        ":test_data/sample_d.json",
        ":test_data/sample_e.json",
        ":test_data/sample_f.json",
        ":test_data/sample_g.json",
        ":test_data/valid.json",
        ":test_data/invalid.json",
    ],
    deps = [
        ":test_proto",
        "//engine/core",
        "//engine/gems/serialization",
        "//engine/gems/serialization:base64",
        "//engine/gems/serialization:capnp",
        "//engine/gems/serialization:json",
        "@gtest//:main",
    ],
)

cc_capnp_library(
    name = "test_proto",
    protos = ["test.capnp"],
)
