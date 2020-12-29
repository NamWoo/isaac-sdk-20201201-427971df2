"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:cc_cuda_library.bzl", "cc_cuda_library")
load("//bzl:isaac_engine.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "image",
    srcs = [
        "color.cpp",
        "conversions.cpp",
        "io.cpp",
        "processing.cpp",
        "utils.cpp",
    ],
    hdrs = [
        "color.hpp",
        "conversions.hpp",
        "distance_map.hpp",
        "io.hpp",
        "processing.hpp",
        "utils.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/core/image",
        "//engine/core/tensor",
        "//engine/gems/algorithm:string_utils",
        "//engine/gems/math",
        "//engine/gems/image:cuda_conversions",
        "//engine/gems/interpolation:linear",
        "//third_party/nlohmann",
        # TODO: Remove this dependencies by creating two libs.
        "@libjpeg//:jpeg",
        "@org_libpng_libpng//:libpng",
    ],
)

cc_cuda_library(
    name = "cuda_conversions",
    srcs = [
        "cuda/image_to_tensor.cu.cpp",
        "cuda/yuv2rgb.cu.cpp",
    ],
    hdrs = [
        "cuda/image_to_tensor.cu.hpp",
        "cuda/yuv2rgb.cu.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/cuda_utils",
        "//third_party:cudart",
    ],
)

isaac_cc_library(
    name = "cuda_utils",
    srcs = [
        "cuda_utils.cpp",
    ],
    hdrs = [
        "cuda_utils.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":image",
        "//engine/core",
        "//engine/core/image",
        "//engine/core/tensor",
        "//third_party:cudart",
        "//third_party:nppial",
        "//third_party:nppidei",
        "//third_party:nppif",
        "//third_party:nppig",
        "//third_party:npps",
    ],
)
