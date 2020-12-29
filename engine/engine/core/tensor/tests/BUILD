"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
cc_test(
    name = "element_type",
    size = "small",
    srcs = ["element_type.cpp"],
    deps = [
        "//engine/core/tensor:element_type",
        "@gtest//:main",
    ],
)

cc_test(
    name = "tensor",
    size = "small",
    srcs = ["tensor.cpp"],
    deps = [
        "//engine/core/tensor",
        "//engine/gems/math",
        "@gtest//:main",
    ],
)

cc_test(
    name = "sample_cloud",
    size = "small",
    srcs = ["sample_cloud.cpp"],
    deps = [
        "//engine/core/tensor:sample_cloud",
        "@gtest//:main",
    ],
)

cc_test(
    name = "universal_tensor",
    size = "small",
    srcs = ["universal_tensor.cpp"],
    deps = [
        "//engine/core/tensor:universal_tensor",
        "@gtest//:main",
    ],
)
