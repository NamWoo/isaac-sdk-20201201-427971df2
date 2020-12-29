"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "alice",
    visibility = ["//visibility:public"],
    deps = [
        ":impl",
        "//engine/alice/tools:gather_component_info",
    ],
)

isaac_cc_library(
    name = "message",
    srcs = ["message.cpp"],
    hdrs = ["message.hpp"],
    linkopts = [
        "-lstdc++fs",
        "-ldl",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/buffers",
        "//engine/gems/serialization:capnp",
        "//engine/gems/serialization:json",
        "//engine/gems/uuid",
    ],
)

isaac_cc_library(
    name = "impl",
    srcs = glob(
        [
            "backend/*.cpp",
            "components/*.cpp",
            "hooks/*.cpp",
            "*.cpp",
        ],
        exclude = ["message.cpp"],
    ),
    hdrs = glob(
        [
            "backend/*.hpp",
            "components/*.hpp",
            "hooks/*.hpp",
            "*.hpp",
        ],
        exclude = ["message.hpp"],
    ),
    linkopts = [
        "-lstdc++fs",
        "-ldl",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":message",
        "//engine/alice/utils:path_utils",
        "//engine/core",
        "//engine/core/buffers",
        "//engine/core/math",
        "//engine/gems/algorithm:string_utils",
        "//engine/gems/math",
        "//engine/gems/pose_tree",
        "//engine/gems/scheduler",
        "//engine/gems/serialization",
        "//engine/gems/serialization:capnp",
        "//engine/gems/serialization:json",
        "//engine/gems/sight",
        "//engine/gems/system:filesystem",
        "//engine/gems/uuid",
        "//gxf/gems/staging_queue",
        "@breakpad",
        "@capnproto//:capnp_lite",
        "@com_google_absl//absl/strings",
        "@redis//:hiredis",
    ],
    alwayslink = True,
)
