"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "x86_64",
    srcs = [
        "x86_64/libhgmm.a",
        "x86_64/libhgmm_cuda.a",
    ],
    hdrs = [
        "hgmm_impl/hgmm.hpp",
        "hgmm_impl/segment.hpp",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "aarch64_jp44",
    srcs = [
        "aarch64_jp44/libhgmm.a",
        "aarch64_jp44/libhgmm_cuda.a",
    ],
    hdrs = [
        "hgmm_impl/hgmm.hpp",
        "hgmm_impl/segment.hpp",
    ],
    visibility = ["//visibility:public"],
)
