"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:isaac_engine.bzl", "isaac_cc_binary", "isaac_cc_library")

isaac_cc_library(
    name = "sight",
    srcs = [
        "sight.cpp",
    ],
    hdrs = [
        "sight.hpp",
        "sight_interface.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":sop",
        "//engine/core",
        "//engine/core/image",
        "//engine/gems/image",
        "//engine/gems/math",
    ],
)

isaac_cc_library(
    name = "sop",
    hdrs = ["sop.hpp",],
    visibility = ["//visibility:public"],
    deps = [
        ":sop_asset",
        ":sop_image",
        ":sop_text",
        ":sop_style",
        ":sop_transform",
        ":named_sop",
        ":serialize",
        ":sop_point_cloud",
        "//engine/core",
        "//engine/gems/image",
        "//engine/gems/math",
        "//engine/gems/serialization:json",
    ],
)

isaac_cc_library(
    name = "serialize",
    hdrs = ["serialize.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/gems/geometry:types",
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "sop_point_cloud",
    srcs = ["sop_point_cloud.cpp"],
    hdrs = ["sop_point_cloud.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/tensor:sample_cloud",
        "//engine/gems/serialization:base64",
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "utils",
    srcs = ["utils.cpp"],
    hdrs = ["utils.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":sop",
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "named_sop",
    srcs = ["named_sop.cpp"],
    hdrs = ["named_sop.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "sop_transform",
    srcs = ["sop_transform.cpp"],
    hdrs = ["sop_transform.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/gems/geometry:pinhole",
        "//engine/gems/geometry:types",
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "sop_style",
    srcs = ["sop_style.cpp"],
    hdrs = ["sop_style.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/image",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "sop_image",
    srcs = ["sop_image.cpp"],
    hdrs = ["sop_image.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/image",
        "//engine/gems/image",
        "//engine/gems/serialization:base64",
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "sop_text",
    srcs = ["sop_text.cpp"],
    hdrs = ["sop_text.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core/math",
        "//engine/gems/serialization:json",
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "sop_asset",
    srcs = ["sop_asset.cpp"],
    hdrs = ["sop_asset.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/nlohmann",
    ],
)

isaac_cc_library(
    name = "kinematics_json",
    srcs = [
        "kinematics_json.cpp",
    ],
    hdrs = [
        "kinematics_json.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/gems/image",
        "//engine/gems/serialization:json",
        "//engine/gems/sight",
        "//third_party/nlohmann",
    ],
)
