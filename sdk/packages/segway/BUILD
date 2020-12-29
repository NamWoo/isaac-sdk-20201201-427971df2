"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "segway",
    deps = [
        ":segway_rmp_driver",
    ],
)

isaac_component(
    name = "segway_rmp_driver",
    deps = [
        "//messages/state:differential_base",
        "//packages/engine_gems/state:io",
        "//packages/segway/gems:segway",
    ],
)
