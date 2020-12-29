"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_test(
    name = "staging_queue",
    srcs = ["test_staging_queue.cpp"],
    size = "small",
    deps = [
      "//gxf/gems/staging_queue",
      "@gtest//:main",
    ],
)

cc_test(
    name = "staging_queue_iterator",
    srcs = ["test_staging_queue_iterator.cpp"],
    size = "small",
    deps = [
      "//gxf/gems/staging_queue:staging_queue_iterator",
      "@gtest//:main",
    ],
)
