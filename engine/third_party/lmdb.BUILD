"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

package(default_visibility = ["//visibility:public"])

licenses(["notice"])  # The OpenLDAP Public License

cc_library(
    name = "lmdb",
    srcs = [
        "libraries/liblmdb/mdb.c",
        "libraries/liblmdb/midl.c",
    ],
    hdrs = [
        "libraries/liblmdb/lmdb.h",
        "libraries/liblmdb/midl.h",
    ],
    copts = [
        "-Wno-unused-but-set-variable",
    ],
    includes = ["libraries/"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "mdb_stat",
    srcs = ["libraries/liblmdb/mdb_stat.c"],
    linkopts = ["-pthread"],
    deps = [":lmdb"],
)
