"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "motors",
    srcs = [
        "constant_motor.cpp",
        "prismatic_motor.cpp",
        "revolute_motor.cpp",
    ],
    hdrs = [
        "constant_motor.hpp",
        "motor.hpp",
        "prismatic_motor.hpp",
        "revolute_motor.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/core/math:dual_quaternion",
    ],
)

isaac_cc_library(
    name = "kinematic_tree",
    srcs = ["kinematic_tree.cpp"],
    hdrs = ["kinematic_tree.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":motors",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/math",
    ],
)

isaac_cc_library(
    name = "kinematic",
    srcs = ["kinematic.cpp"],
    hdrs = ["kinematic.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":kinematic_tree",
        ":motors",
        "//packages/math/gems/optimization",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/core/math:dual_quaternion",
    ],
)

isaac_cc_library(
    name = "json_loader",
    srcs = ["json_loader.cpp"],
    hdrs = ["json_loader.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":kinematic_tree",
        ":motors",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/math",
        "@com_nvidia_isaac_engine//engine/core/math:dual_quaternion",
        "@com_nvidia_isaac_engine//engine/gems/serialization:json",
    ],
)
