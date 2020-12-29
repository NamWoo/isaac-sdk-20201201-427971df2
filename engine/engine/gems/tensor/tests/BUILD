"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "copy",
    size = "small",
    srcs = ["copy.cpp"],
    deps = [
        "//engine/gems/tensor",
        "@gtest//:main",
    ],
)

cc_test(
    name = "transpose",
    size = "small",
    srcs = ["transpose.cpp"],
    deps = [
        "//engine/gems/tensor:transpose",
        "@gtest//:main",
    ],
)
