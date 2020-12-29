"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library", "isaac_cc_test_group")

isaac_cc_test_group(
    srcs = [
        "color.cpp",
        "distance_map.cpp",
    ],
    deps = ["//engine/gems/image"],
)

cc_test(
    name = "conversions",
    size = "small",
    srcs = ["conversions.cpp"],
    data = ["//engine/gems/image/data"],
    deps = [
        "//engine/gems/image",
        "//engine/gems/tensor",
        "@gtest//:main",
    ],
)

cc_test(
    name = "io",
    size = "small",
    srcs = ["io.cpp"],
    data = ["//engine/gems/image/data"],
    deps = [
        "//engine/gems/image",
        "@gtest//:main",
    ],
)

cc_test(
    name = "utils",
    size = "small",
    srcs = ["utils.cpp"],
    data = ["//engine/gems/image/data"],
    deps = [
        "//engine/gems/image",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cuda_utils",
    size = "small",
    srcs = ["cuda_utils.cpp"],
    data = ["//engine/gems/image/data"],
    deps = [
        "//engine/gems/image:cuda_utils",
        "//engine/gems/tensor",
        "//third_party:nppig",
        "@gtest//:main",
    ],
)
