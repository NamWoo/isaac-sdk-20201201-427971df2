"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_cc_module", "isaac_component")

isaac_component(
    name = "evaluator",
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "frame_rate_evaluator",
    deps = [":evaluator"],
    visibility = ["//visibility:public"],
)

isaac_component(
    name = "color_sanity_evaluator",
    deps = [":evaluator"],
    visibility = ["//visibility:public"]
)

isaac_component(
    name = "depth_sanity_evaluator",
    deps = [":evaluator"],
    visibility = ["//visibility:public"]
)

isaac_component(
    name = "depth_evaluator",
    deps = [":evaluator"],
    visibility = ["//visibility:public"]
)

isaac_component(
    name = "color_evaluator",
    deps = [
        ":evaluator",
        "//third_party:opencv",
    ],
    visibility = ["//visibility:public"]
)

isaac_component(
    name = "april_tags_evaluator",
    deps = [
        ":evaluator",
    ],
    visibility = ["//visibility:public"]
)

isaac_cc_module(
    name = "evaluators",
    deps = [
        "frame_rate_evaluator",
        "color_sanity_evaluator",
        "depth_sanity_evaluator",
        "depth_evaluator",
        "color_evaluator",
        "april_tags_evaluator",
    ],
)
