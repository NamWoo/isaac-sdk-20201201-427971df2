"""
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build/style:cpplint.bzl", "cpplint")

def _combine(srcs, hdrs):
    if srcs == None:
        srcs = []
    if hdrs == None:
        hdrs = []
    return srcs + hdrs

def _shall_lint(tags):
    return tags == None or "nolint" not in tags

def isaac_cc_binary(name, srcs = None, hdrs = None, tags = None, **kwargs):
    """
    A standard cc_binary with lint support
    """

    native.cc_binary(name = name, srcs = srcs, hdrs = hdrs, tags = tags, **kwargs)

    if _shall_lint(tags):
        cpplint(name = name, srcs = _combine(srcs, hdrs))

def isaac_cc_library(
        name,
        srcs = None,
        hdrs = None,
        deps = None,
        tags = None,
        visibility = None,
        **kwargs):
    """
    A standard cc_library with lint support
    """

    native.cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps,
        tags = tags,
        visibility = visibility,
        **kwargs
    )

    if _shall_lint(tags):
        cpplint(name = name, srcs = _combine(srcs, hdrs))

def isaac_cc_test_group(srcs, deps = [], size = "small", copts = [], **kwargs):
    """
    Creates on cc_test target per source file given in `srcs`. The test is given the same name as
    the corresponding source file. Only '*.cpp' files are supported. Every test will have the same
    dependencies `deps`. The gtest dependency is added automatically.
    """
    for src in srcs:
        if not src.endswith(".cpp"):
            fail("Only cpp files are allowed as tests")
        native.cc_test(
            name = src[:-4],
            size = size,
            srcs = [src],
            deps = deps + ["@gtest//:main"],
            copts = copts + ["-Wno-unused-but-set-variable"],
            **kwargs
        )
