"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "object_detection_utils_tests",
    size = "small",
    srcs = ["object_detection_utils.cpp"],
    deps = [
        "@com_nvidia_isaac_engine//engine/gems/geometry:n_cuboid",
        "//packages/ml/gems:object_detection_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "cosine_similarity_tests",
    size = "small",
    srcs = ["cosine_similarity.cpp"],
    deps = [
        "//packages/ml/gems:cosine_similarity",
        "@gtest//:main",
    ],
)
