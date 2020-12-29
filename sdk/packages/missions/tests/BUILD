"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

py_test(
    name = "mission_test",
    srcs = ["mission_test.py"],
    data = [
        "//packages/behavior_tree/apps:missions",
    ],
    tags = ["host"],
    deps = [
        "//packages/missions",
        "//packages/pyalice",
    ],
)
