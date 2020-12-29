"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load(
    "//bzl:deps.bzl",
    "isaac_http_archive",
    "isaac_new_git_repository",
)

def clean_dep(dep):
    return str(Label(dep))

def isaac_engine_workspace():
    """Loads external dependencies required to build apps with alice"""

    isaac_http_archive(
        name = "gtest",
        build_file = clean_dep("//third_party:gtest.BUILD"),
        sha256 = "d88ad7eba129d2d5453da05c6318af6babf65af37835d720e6bfa105d61cf5ce",
        url = "https://developer.nvidia.com/isaac/download/third_party/googletest-release-1-8-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "googletest-release-1.8.0/googletest",
        licenses = ["@gtest//:LICENSE"],
    )

    isaac_http_archive(
        name = "benchmark",
        build_file = clean_dep("//third_party:benchmark.BUILD"),
        sha256 = "f19559475a592cbd5ac48b61f6b9cedf87f0b6775d1443de54cfe8f53940b28d",
        url = "https://developer.nvidia.com/isaac/download/third_party/benchmark-1-3-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "benchmark-1.3.0",
        licenses = ["@benchmark//:LICENSE"],
    )

    isaac_http_archive(
        # Name is matching that of deps for cartographer (@cartographer//bazel:repositories.bzl)
        name = "net_zlib_zlib",
        build_file = clean_dep("//third_party:zlib.BUILD"),
        sha256 = "c3e5e9fdd5004dcb542feda5ee4f0ff0744628baf8ed2dd5d66f8ca1197cb1a1",
        url = "https://developer.nvidia.com/isaac/download/third_party/zlib-1-2-11-tar-gz",
        type = "tar.gz",
        strip_prefix = "zlib-1.2.11",
        licenses = ["https://zlib.net/zlib_license.html"],
    )

    isaac_http_archive(
        name = "boringssl",
        sha256 = "524ba98a56300149696481b4cb9ddebd0c7b7ac9b9f6edee81da2d2d7e5d2bb3",
        url = "https://developer.nvidia.com/isaac/download/third_party/boringssl-a0fb951d2a26a8ee746b52f3ba81ab011a0af778-tar-gz",
        type = "tar.gz",
        patches = [clean_dep("//third_party:boringssl.patch")],
        strip_prefix = "boringssl-a0fb951d2a26a8ee746b52f3ba81ab011a0af778",
        licenses = ["@boringssl//:LICENSE"],
    )

    isaac_http_archive(
        name = "libuuid",
        build_file = clean_dep("//third_party:uuid.BUILD"),
        sha256 = "46af3275291091009ad7f1b899de3d0cea0252737550e7919d17237997db5644",
        url = "https://developer.nvidia.com/isaac/download/third_party/libuuid-1-0-3-tar-gz",
        type = "tar.gz",
        strip_prefix = "libuuid-1.0.3",
        licenses = ["@libuuid//:COPYING"],
    )

    isaac_new_git_repository(
        # Name is matching that of deps for cartographer (@cartographer//bazel:repositories.bzl)
        name = "org_tuxfamily_eigen",
        build_file = clean_dep("//third_party:eigen.BUILD"),
        commit = "21ae2afd4edaa1b69782c67a54182d34efe43f9c",  # tag 3.3.7
        # Patch to fix some warnings:
        #  Eigen/Core was including `host_defines.h` directly instead of `cuda_runtime_api.h`
        patches = [clean_dep("//third_party:eigen.patch")],
        remote = "https://gitlab.com/libeigen/eigen.git",
        licenses = [
            "@org_tuxfamily_eigen/COPYING.BSD",
            "@org_tuxfamily_eigen/COPYING.MPL2",
            "@org_tuxfamily_eigen/COPYING.README",
        ],
    )

    isaac_http_archive(
        # Name is matching that of deps for cartographer (@cartographer//bazel:repositories.bzl)
        name = "org_libpng_libpng",
        build_file = clean_dep("//third_party:png.BUILD"),
        sha256 = "2f1e960d92ce3b3abd03d06dfec9637dfbd22febf107a536b44f7a47c60659f6",
        url = "https://developer.nvidia.com/isaac/download/third_party/libpng-1-6-34-tar-xz",
        type = "tar.xz",
        strip_prefix = "libpng-1.6.34",
        licenses = ["@org_libpng_libpng//:libpng-LICENSE.txt"],
    )

    isaac_http_archive(
        name = "capnproto",
        build_file = clean_dep("//third_party:capnproto.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/capnproto-0-7-0-tar-gz",
        type = "tar.gz",
        sha256 = "76c7114a3d142ad08b7208b3964a26e72a6320ee81331d3f0b87569fc9c47a28",
        strip_prefix = "capnproto-0.7.0",
        licenses = ["@capnproto//:LICENSE"],
    )

    isaac_http_archive(
        name = "asio",
        build_file = clean_dep("//third_party:asio.BUILD"),
        sha256 = "53672dcf3cf3394da10b32880de88b30e5787a04d121aa5dd92d4f3e577cf78e",
        url = "https://developer.nvidia.com/isaac/download/third_party/asio-1-16-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "asio-1.16.0",
        licenses = ["@asio//:asio-1.16.0/LICENSE_1_0.txt"],
    )

    isaac_http_archive(
        name = "com_github_gflags_gflags",
        url = "https://developer.nvidia.com/isaac/download/third_party/gflags-e292e0452fcfd5a8ae055b59052fc041cbab4abf-tar-gz",
        type = "tar.gz",
        sha256 = "a4c5171355e67268b4fd2f31c3f7f2d125683d12e0686fc14893a3ca8c803659",
        licenses = ["@com_github_gflags_gflags//:COPYING.txt"],
    )

    native.bind(
        name = "gflags",
        actual = "@com_github_gflags_gflags//:gflags",
    )

    isaac_http_archive(
        name = "lmdb",
        build_file = clean_dep("//third_party:lmdb.BUILD"),
        sha256 = "44602436c52c29d4f301f55f6fd8115f945469b868348e3cddaf91ab2473ea26",
        url = "https://developer.nvidia.com/isaac/download/third_party/lmdb-LMDB_0-9-24-tar-gz",
        type = "tar.gz",
        strip_prefix = "lmdb-LMDB_0.9.24",
        licenses = ["@lmdb//:libraries/liblmdb/LICENSE"],
    )

    isaac_http_archive(
        name = "nasm",
        build_file = clean_dep("//third_party:nasm.BUILD"),
        sha256 = "00b0891c678c065446ca59bcee64719d0096d54d6886e6e472aeee2e170ae324",
        url = "https://developer.nvidia.com/isaac/download/third_party/nasm-2-12-02-tar-bz2",
        type = "tar.bz2",
        strip_prefix = "nasm-2.12.02",
        licenses = ["@nasm//:LICENSE"],
    )

    isaac_http_archive(
        # Name is matching that of deps for cartographer (@cartographer//bazel:repositories.bzl)
        name = "libjpeg",
        build_file = clean_dep("//third_party:jpeg.BUILD"),
        sha256 = "c15a9607892113946379ccea3ca8b85018301b200754f209453ab21674268e77",
        url = "https://developer.nvidia.com/isaac/download/third_party/libjpeg-turbo-1-5-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "libjpeg-turbo-1.5.1",
        licenses = ["@libjpeg//:LICENSE.md"],
    )

    isaac_http_archive(
        name = "redis",
        build_file = clean_dep("//third_party:redis.BUILD"),
        sha256 = "2b38bf9ce0550410341b281e901830beac1fa2ca89876dd04867f094be1489a3",
        url = "https://developer.nvidia.com/isaac/download/third_party/redis-5-0-7-zip",
        type = "zip",
        strip_prefix = "redis-5.0.7",
        licenses = ["@redis//:COPYING"],
    )

    isaac_http_archive(
        name = "uwebsockets",
        build_file = clean_dep("//third_party:uwebsockets.BUILD"),
        sha256 = "663a22b521c8258e215e34e01c7fcdbbd500296aab2c31d36857228424bb7675",
        url = "https://developer.nvidia.com/isaac/download/third_party/uwebsockets-0-14-8-tar-gz",
        type = "tar.gz",
        patches = [clean_dep("//third_party:uwebsockets.patch")],
        strip_prefix = "uWebSockets-0.14.8",
        licenses = ["@uwebsockets//:LICENSE"],
    )

    isaac_http_archive(
        name = "snappy",
        build_file = clean_dep("//third_party:snappy.BUILD"),
        sha256 = "3dfa02e873ff51a11ee02b9ca391807f0c8ea0529a4924afa645fbf97163f9d4",
        url = "https://developer.nvidia.com/isaac/download/third_party/snappy-1-1-7-tar-gz",
        type = "tar.gz",
        strip_prefix = "snappy-1.1.7",
        licenses = ["@snappy//:COPYING"],
    )

    isaac_http_archive(
        name = "curl",
        build_file = clean_dep("//third_party:curl.BUILD"),
        sha256 = "ff3e80c1ca6a068428726cd7dd19037a47cc538ce58ef61c59587191039b2ca6",
        url = "https://developer.nvidia.com/isaac/download/third_party/curl-7-49-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "curl-7.49.1",
        licenses = ["@curl//:COPYING"],
    )

    isaac_http_archive(
        name = "lss",
        build_file = clean_dep("//third_party:lss.BUILD"),
        sha256 = "6d2e98e9d360797db6348ae725be901c1947e5736d87f07917c2bd835b03eeef",
        url = "https://developer.nvidia.com/isaac/download/third_party/linux-syscall-support-93426bda6535943ff1525d0460aab5cc0870ccaf-tar-gz",
        type = "tar.gz",
        licenses = ["@lss//:linux_syscall_support.h"],
    )

    isaac_http_archive(
        name = "breakpad",
        build_file = clean_dep("//third_party:breakpad.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/breakpad-13c1568702e8804bc3ebcfbb435a2786a3e335cf-tar-gz",
        type = "tar.gz",
        sha256 = "9420c263a0db0a0e07a789589f46f0d69b72e921e7eeabb4af3b0018043e5225",
        licenses = ["@breakpad//:LICENSE"],
    )

    isaac_http_archive(
        name = "nvcc_10",
        build_file = clean_dep("//third_party:nvcc_10.BUILD"),
        sha256 = "eb768cd2d79f2431bbdd51a6fc5e9390ff288d26d1fdbc800362840fffce7ba0",
        url = "https://developer.nvidia.com/isaac/download/third_party/cuda_10_2_nvcc-tar-xz",
        type = "tar.xz",
        licenses = ["http://docs.nvidia.com/cuda/eula/index.html"],
    )

    isaac_http_archive(
        name = "cuda_x86_64",
        build_file = clean_dep("//third_party:cuda_x86_64.BUILD"),
        sha256 = "82dcd6f677f075e59d8a619822a7a4b0f04bbb5d2dbfb5bc99743c7cb8ad35d1",
        url = "https://developer.nvidia.com/isaac/download/third_party/cuda-10-2-cudnn-8-0-3-x86_64-tar-bz2",
        type = "tar.bz2",
        licenses = ["http://docs.nvidia.com/cuda/eula/index.html"],
    )

    isaac_http_archive(
        name = "cuda_aarch64_jetpack44",
        build_file = clean_dep("//third_party:cuda_aarch64_jetpack44.BUILD"),
        sha256 = "53e0a2f45088878f00f3913070ab6d57efedc8b19fbf9d36564967c99a1dfe75",
        url = "https://developer.nvidia.com/isaac/download/third_party/cuda_10_2_cudnn_8_jp44_ga_hdr-tar-bz2",
        type = "tar.bz2",
        licenses = ["http://docs.nvidia.com/cuda/eula/index.html"],
    )

    isaac_http_archive(
        name = "com_google_absl",
        sha256 = "c8ba586a9ab12bc4a67bb419fc0d2146200942b072bac95f50490f977b7fb04f",
        strip_prefix = "abseil-cpp-5441bbe1db5d0f2ca24b5b60166367b0966790af",
        url = "https://github.com/abseil/abseil-cpp/archive/5441bbe1db5d0f2ca24b5b60166367b0966790af.tar.gz",
        licenses = ["https://github.com/abseil/abseil-cpp/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "nasm",
        build_file = clean_dep("//third_party:nasm.BUILD"),
        sha256 = "00b0891c678c065446ca59bcee64719d0096d54d6886e6e472aeee2e170ae324",
        url = "https://developer.nvidia.com/isaac/download/third_party/nasm-2-12-02-tar-bz2",
        type = "tar.bz2",
        strip_prefix = "nasm-2.12.02",
        licenses = ["@nasm//:LICENSE"],
    )

    isaac_http_archive(
        # Name is matching that of deps for cartographer (@cartographer//bazel:repositories.bzl)
        name = "libjpeg",
        build_file = clean_dep("//third_party:jpeg.BUILD"),
        sha256 = "c15a9607892113946379ccea3ca8b85018301b200754f209453ab21674268e77",
        url = "https://developer.nvidia.com/isaac/download/third_party/libjpeg-turbo-1-5-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "libjpeg-turbo-1.5.1",
        licenses = ["@libjpeg//:LICENSE.md"],
    )
