"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
licenses(["notice"])  # BSD 3-Clause

exports_files(["COPYING"])

genrule(
    name = "make_release_header",
    outs = [
        "release.h",
    ],
    cmd = "cat <<'EOF' >$@\n" +
          "#define REDIS_GIT_SHA1 \"0\"\n" +
          "#define REDIS_GIT_DIRTY \"0\"\n" +
          "#define REDIS_BUILD_ID \"0\"\n" +
          "EOF",
)

cc_library(
    name = "lua",
    srcs = glob(["deps/lua/src/*.h", "deps/lua/src/*.c", "src/solarisfixes.h"]),
    includes = ["deps/lua/src"],
    copts = [
        "-Wno-misleading-indentation",
        "-Wno-unused-variable",
    ],
)

cc_library(
    name = "hiredis",
    srcs = [
        "deps/hiredis/async.c",
        "deps/hiredis/async.h",
        "deps/hiredis/dict.c",
        "deps/hiredis/dict.h",
        "deps/hiredis/fmacros.h",
        "deps/hiredis/hiredis.c",
        "deps/hiredis/hiredis.h",
        "deps/hiredis/net.c",
        "deps/hiredis/net.h",
        "deps/hiredis/read.c",
        "deps/hiredis/read.h",
        "deps/hiredis/sdsalloc.h",
        "deps/hiredis/sds.c",
        "deps/hiredis/sds.h",
    ],
    hdrs = ["deps/hiredis/hiredis.h"],
    includes = ["deps/hiredis"],
    visibility = ["//visibility:public"],
    textual_hdrs = ["deps/hiredis/dict.c"],
    copts = [
        "-U_DEFAULT_SOURCE",
        "-Wno-unused-function",
    ],
)

cc_library(
    name = "linenoise",
    srcs = ["deps/linenoise/linenoise.c"],
    hdrs = ["deps/linenoise/linenoise.h"],
    includes = ["deps/linenoise"],
)

cc_binary(
    name = "redis-server",
    deps = ["redis-server-lib"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "redis-server-lib",
    deps = [
        "lua",
        "hiredis",
        "linenoise",
    ],
    textual_hdrs = ["src/ae_epoll.c"],
    copts = [
        "-U_DEFAULT_SOURCE",
        "-Wno-misleading-indentation",
        "-Wno-unused-variable",
        "-Wno-unused-function",
    ],
    linkopts = [
        "-ldl",
        "-lpthread",
    ],
    hdrs = glob(["src/*.h"]),
    srcs = glob(["src/*.h"]) + [
        "release.h",
        "src/adlist.c",
        "src/quicklist.c",
        "src/ae.c",
        "src/anet.c",
        "src/dict.c",
        "src/server.c",
        "src/sds.c",
        "src/zmalloc.c",
        "src/lzf_c.c",
        "src/lzf_d.c",
        "src/pqsort.c",
        "src/zipmap.c",
        "src/sha1.c",
        "src/ziplist.c",
        "src/release.c",
        "src/networking.c",
        "src/util.c",
        "src/object.c",
        "src/db.c",
        "src/replication.c",
        "src/rdb.c",
        "src/t_string.c",
        "src/t_list.c",
        "src/t_set.c",
        "src/t_zset.c",
        "src/t_hash.c",
        "src/config.c",
        "src/aof.c",
        "src/pubsub.c",
        "src/multi.c",
        "src/debug.c",
        "src/sort.c",
        "src/intset.c",
        "src/syncio.c",
        "src/cluster.c",
        "src/crc16.c",
        "src/endianconv.c",
        "src/slowlog.c",
        "src/scripting.c",
        "src/bio.c",
        "src/rio.c",
        "src/rand.c",
        "src/memtest.c",
        "src/crc64.c",
        "src/bitops.c",
        "src/sentinel.c",
        "src/notify.c",
        "src/setproctitle.c",
        "src/blocked.c",
        "src/hyperloglog.c",
        "src/latency.c",
        "src/sparkline.c",
        "src/redis-check-rdb.c",
        "src/redis-check-aof.c",
        "src/geo.c",
        "src/lazyfree.c",
        "src/module.c",
        "src/evict.c",
        "src/expire.c",
        "src/geohash.c",
        "src/geohash_helper.c",
        "src/childinfo.c",
        "src/defrag.c",
        "src/siphash.c",
        "src/rax.c",
        "src/t_stream.c",
        "src/listpack.c",
        "src/localtime.c",
        "src/lolwut.c",
        "src/lolwut5.c",
    ],
    includes = ["src"],
)
