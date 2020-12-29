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
    name = "allocator",
    visibility = ["//visibility:public"],
    deps = [
        "allocator_base",
        "allocators",
        "cached_allocator",
        "cuda_malloc_allocator",
        "malloc_allocator",
    ],
)

isaac_cc_library(
    name = "allocator_base",
    hdrs = ["allocator_base.hpp"],
    visibility = ["//visibility:public"],
    srcs = ["allocator_base.cpp"],
    deps = ["//engine/core"],
)

isaac_cc_library(
    name = "allocators",
    hdrs = ["allocators.hpp"],
    visibility = ["//visibility:public"],
    srcs = ["allocators.cpp"],
    deps = [
        "allocator_base",
        "cached_allocator",
        "cuda_malloc_allocator",
        "malloc_allocator",
    ],
)

isaac_cc_library(
    name = "cached_allocator",
    hdrs = ["cached_allocator.hpp"],
    visibility = ["//visibility:public"],
    srcs = ["cached_allocator.cpp"],
    deps = [
        "allocator_base",
        "//engine/core",
        "@boost//:lockfree",
    ],
)

isaac_cc_library(
    name = "cuda_malloc_allocator",
    hdrs = ["cuda_malloc_allocator.hpp"],
    visibility = ["//visibility:public"],
    srcs = ["cuda_malloc_allocator.cpp"],
    deps = [
        "allocator_base",
        "//engine/core",
        "//third_party:cudart",
    ],
)

isaac_cc_library(
    name = "malloc_allocator",
    hdrs = ["malloc_allocator.hpp"],
    visibility = ["//visibility:public"],
    srcs = ["malloc_allocator.cpp"],
    deps = ["allocator_base"],
)

isaac_cc_library(
    name = "test_utils",
    hdrs = ["test_utils.hpp"],
    visibility = ["//visibility:public"],
    srcs = ["test_utils.cpp"],
    deps = ["allocator_base"],
)
