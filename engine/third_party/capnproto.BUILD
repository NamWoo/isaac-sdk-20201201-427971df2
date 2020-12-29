"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "capnproto",
    visibility = ["//visibility:public"],
    deps = [
        ":capnp_lite",
        ":kj_lite",
    ],
)

cc_library(
    name = "kj_lite",
    srcs = [
        "c++/src/kj/arena.c++",
        "c++/src/kj/array.c++",
        "c++/src/kj/common.c++",
        "c++/src/kj/debug.c++",
        "c++/src/kj/encoding.c++",
        "c++/src/kj/exception.c++",
        "c++/src/kj/hash.c++",
        "c++/src/kj/io.c++",
        "c++/src/kj/main.c++",
        "c++/src/kj/memory.c++",
        "c++/src/kj/mutex.c++",
        "c++/src/kj/string.c++",
        "c++/src/kj/string-tree.c++",
        "c++/src/kj/table.c++",
        "c++/src/kj/test-helpers.c++",
        "c++/src/kj/thread.c++",
        "c++/src/kj/time.c++",
        "c++/src/kj/units.c++",
    ],
    hdrs = glob([
        "c++/src/kj/*.h",
    ]),
    copts = [
        "-Wno-sign-compare",
        "-Wno-strict-aliasing",
    ],
    defines = [
        "KJ_HEADER_WARNINGS",
    ],
    includes = [
        ".",
        "c++/src",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "kj_heavy",
    srcs = [
        "c++/src/kj/filesystem.c++",
        "c++/src/kj/filesystem-disk-unix.c++",
        "c++/src/kj/parse/char.c++",
        "c++/src/kj/refcount.c++",
        "c++/src/kj/string-tree.c++",
    ],
    hdrs = glob([
        "c++/src/kj/*.h",
        "c++/src/kj/parse/*.h",
    ]),
    copts = [
        "-Wno-sign-compare",
    ],
    defines = [
        "KJ_HEADER_WARNINGS",
    ],
    includes = [
        ".",
        "c++/src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":kj_lite",
    ],
)

cc_library(
    name = "capnp_lite",
    srcs = [
        "c++/src/capnp/any.c++",
        "c++/src/capnp/arena.c++",
        "c++/src/capnp/blob.c++",
        "c++/src/capnp/c++.capnp.c++",
        "c++/src/capnp/compat/json.c++",
        "c++/src/capnp/compat/json.capnp.c++",
        "c++/src/capnp/dynamic.c++",
        "c++/src/capnp/layout.c++",
        "c++/src/capnp/list.c++",
        "c++/src/capnp/message.c++",
        "c++/src/capnp/schema.c++",
        "c++/src/capnp/schema.capnp.c++",
        "c++/src/capnp/serialize.c++",
        "c++/src/capnp/serialize-packed.c++",
    ],
    hdrs = glob([
        "c++/src/capnp/*.h",
        "c++/src/capnp/compat/json.capnp.h",
        "c++/src/capnp/compat/json.h",
    ]),
    copts = [
        "-Wno-sign-compare",
    ],
    defines = [
        "CAPNP_HEADER_WARNINGS",
    ],
    includes = [
        ".",
        "c++/src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":kj_lite",
    ],
)

cc_library(
    name = "capnp_heavy",
    srcs = [
        "c++/src/capnp/schema-loader.c++",
        "c++/src/capnp/stringify.c++",
    ],
    hdrs = glob(["c++/src/capnp/*.h"]),
    copts = [
        "-Wno-sign-compare",
    ],
    defines = [
        "CAPNP_HEADER_WARNINGS",
    ],
    includes = [
        ".",
        "c++/src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":capnp_lite",
        ":kj_heavy",
    ],
)

cc_library(
    name = "capnpc",
    srcs = glob([
        "c++/src/capnp/compiler/md5.c++",
        "c++/src/capnp/compiler/error-reporter.c++",
        "c++/src/capnp/compiler/lexer.capnp.c++",
        "c++/src/capnp/compiler/lexer.c++",
        "c++/src/capnp/compiler/grammar.capnp.c++",
        "c++/src/capnp/compiler/parser.c++",
        "c++/src/capnp/compiler/node-translator.c++",
        "c++/src/capnp/compiler/compiler.c++",
        "c++/src/capnp/compiler/type-id.c++",
        "c++/src/capnp/schema-parser.c++",
        "c++/src/capnp/serialize-text.c++",
    ]),
    hdrs = glob(["c++/src/capnp/compiler/*.h"]),
    copts = [
        "-Wno-maybe-uninitialized",
    ],
    deps = [
        ":capnp_heavy",
    ],
)

cc_binary(
    name = "capnpc_tool",
    srcs = [
        "c++/src/capnp/compiler/capnp.c++",
        "c++/src/capnp/compiler/module-loader.c++",
    ],
    copts = [
        "-Wno-maybe-uninitialized",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":capnp_heavy",
        ":capnpc",
        ":kj_heavy",
    ],
)

cc_binary(
    name = "capnpc-c++",
    srcs = ["c++/src/capnp/compiler/capnpc-c++.c++"],
    visibility = ["//visibility:public"],
    deps = [
        ":capnp_heavy",
        ":kj_heavy",
    ],
)

cc_binary(
    name = "capnpc-capnp",
    srcs = ["c++/src/capnp/compiler/capnpc-capnp.c++"],
    visibility = ["//visibility:public"],
    deps = [
        ":capnp_heavy",
        ":kj_heavy",
    ],
)
