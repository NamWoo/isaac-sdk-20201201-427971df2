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
    name = "parser",
    srcs = [
        "fragment_index.cpp",
        "measure.cpp",
        "parser.cpp",
        "schema.cpp",
    ],
    hdrs = [
        "fragment_index.hpp",
        "measure.hpp",
        "parser.hpp",
        "quantity.hpp",
        "schema.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":typed_composite_view",
        "//messages:composite_proto",
        "//messages:math",
        "//messages:tensor",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/array",
        "@com_nvidia_isaac_engine//engine/core/buffers",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/core/tensor:element_type",
        "@com_nvidia_isaac_engine//engine/gems/algorithm:timeseries",
        "@com_nvidia_isaac_engine//engine/gems/serialization",
    ],
)

isaac_cc_library(
    name = "serializer",
    srcs = [
        "serializer.cpp",
    ],
    hdrs = [
        "serializer.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":parser",
        "//messages:composite_proto",
        "//messages:math",
        "//messages:tensor",
        "@com_nvidia_isaac_engine//engine/core",
        "@com_nvidia_isaac_engine//engine/core/array",
        "@com_nvidia_isaac_engine//engine/core/buffers",
        "@com_nvidia_isaac_engine//engine/core/tensor",
        "@com_nvidia_isaac_engine//engine/core/tensor:element_type",
        "@com_nvidia_isaac_engine//engine/gems/serialization",
    ],
)

isaac_cc_library(
    name = "typed_composite_view",
    hdrs = [
        "typed_composite_view.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac_engine//engine/core",
    ],
)

isaac_cc_library(
    name = "json_formatter",
    hdrs = [
        "json_formatter.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":parser",
        "@com_nvidia_isaac_engine//engine/gems/serialization",
    ],
)
