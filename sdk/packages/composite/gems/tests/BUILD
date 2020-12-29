"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_test_group")

cc_test(
    name = "composite_writer",
    size = "small",
    srcs = ["composite_writer.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/math:test_utils",
        "//packages/composite/gems:parser",
        "//packages/composite/gems:serializer",
        "@gtest//:main",
    ],
)

cc_test(
    name = "composite_from_proto",
    size = "small",
    srcs = ["composite_from_proto.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/tensor",
        "//packages/composite/gems:parser",
        "//packages/composite/gems:serializer",
        "//packages/composite/gems:typed_composite_view",
        "@gtest//:main",
    ],
)

cc_test(
    name = "typed_composite_view",
    size = "small",
    srcs = ["typed_composite_view.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/tensor",
        "//packages/composite/gems:typed_composite_view",
        "@gtest//:main",
    ],
)
