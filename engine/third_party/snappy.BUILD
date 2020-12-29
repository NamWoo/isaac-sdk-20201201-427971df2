"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

package(default_visibility = ["//visibility:public"])

licenses(["notice"])  # BSD 3-Clause

exports_files(["COPYING"])

config_setting(
    name = "windows",
    values = {
        "crosstool_top": "//crosstools/windows",
    },
)

cc_library(
    name = "snappy",
    srcs = [
        "config.h",
        "snappy-c.cc",
        "snappy-c.h",
        "snappy-internal.h",
        "snappy-sinksource.cc",
        "snappy-sinksource.h",
        "snappy-stubs-internal.cc",
        "snappy-stubs-internal.h",
        "snappy-stubs-public.h",
        "snappy.cc",
        "snappy.h",
    ],
    hdrs = ["snappy.h"],
    copts = select({
        ":windows": [
            "/DHAVE_CONFIG_H",
            "/EHsc",
        ],
        "//conditions:default": [
            "-DHAVE_CONFIG_H",
            "-fno-exceptions",
            "-Wno-sign-compare",
            "-Wno-shift-negative-value",
            "-Wno-implicit-function-declaration",
            "-Wno-error",  # some C++ options are applied to C code
        ],
    }),
)

genrule(
    name = "config_h",
    outs = ["config.h"],
    cmd = "\n".join([
        "cat <<'EOF' >$@",
        "#define HAVE_STDDEF_H 1",
        "#define HAVE_STDINT_H 1",
        "",
        "#ifdef __has_builtin",
        "#  if !defined(HAVE_BUILTIN_EXPECT) && __has_builtin(__builtin_expect)",
        "#    define HAVE_BUILTIN_EXPECT 1",
        "#  endif",
        "#  if !defined(HAVE_BUILTIN_CTZ) && __has_builtin(__builtin_ctzll)",
        "#    define HAVE_BUILTIN_CTZ 1",
        "#  endif",
        "#elif defined(__GNUC__) && (__GNUC__ > 3 || __GNUC__ == 3 && __GNUC_MINOR__ >= 4)",
        "#  ifndef HAVE_BUILTIN_EXPECT",
        "#    define HAVE_BUILTIN_EXPECT 1",
        "#  endif",
        "#  ifndef HAVE_BUILTIN_CTZ",
        "#    define HAVE_BUILTIN_CTZ 1",
        "#  endif",
        "#endif",
        "",
        "#ifdef __has_include",
        "#  if !defined(HAVE_BYTESWAP_H) && __has_include(<byteswap.h>)",
        "#    define HAVE_BYTESWAP_H 1",
        "#  endif",
        "#  if !defined(HAVE_UNISTD_H) && __has_include(<unistd.h>)",
        "#    define HAVE_UNISTD_H 1",
        "#  endif",
        "#  if !defined(HAVE_SYS_ENDIAN_H) && __has_include(<sys/endian.h>)",
        "#    define HAVE_SYS_ENDIAN_H 1",
        "#  endif",
        "#  if !defined(HAVE_SYS_MMAN_H) && __has_include(<sys/mman.h>)",
        "#    define HAVE_SYS_MMAN_H 1",
        "#  endif",
        "#  if !defined(HAVE_SYS_UIO_H) && __has_include(<sys/uio.h>)",
        "#    define HAVE_SYS_UIO_H 1",
        "#  endif",
        "#endif",
        "",
        "#ifndef SNAPPY_IS_BIG_ENDIAN",
        "#  ifdef __s390x__",
        "#    define SNAPPY_IS_BIG_ENDIAN 1",
        "#  elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__",
        "#    define SNAPPY_IS_BIG_ENDIAN 1",
        "#  endif",
        "#endif",
        "EOF",
    ]),
)

genrule(
    name = "snappy_stubs_public_h",
    srcs = ["snappy-stubs-public.h.in"],
    outs = ["snappy-stubs-public.h"],
    cmd = ("sed " +
           "-e 's/$${\\(.*\\)_01}/\\1/g' " +
           "-e 's/$${SNAPPY_MAJOR}/1/g' " +
           "-e 's/$${SNAPPY_MINOR}/1/g' " +
           "-e 's/$${SNAPPY_PATCHLEVEL}/7/g' " +
           "$< >$@"),
)
