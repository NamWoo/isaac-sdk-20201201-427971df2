"""
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

_TARGET_PLATFORM = "target_platform"

_GCC_PATH_MAP = {
    "x86_64": "/usr/bin/gcc",
    "jetpack44": "/usr/bin/aarch64-linux-gnu-gcc-7",
}

_NVCC_PATH_MAP = {
    "x86_64": "external/nvcc_10/bin/nvcc",
    "jetpack44": "external/nvcc_10/bin/nvcc",
}

_CUDA_CAPABILITIES_MAP = {
    "x86_64": '"3.5", "5.0", "5.2", "5.3",  "6.0", "6.1", "6.2", "7.0", "7.5"',
    "jetpack44": '"5.3","6.2","7.2"',
}

def _tpl(repository_ctx, tpl, substitutions = {}, out = None):
    repository_ctx.template(
        out,
        Label("//toolchain/crosstool:%s.tpl" % tpl),
        substitutions,
    )

def _toolchain_impl(repository_ctx):
    target_platform = repository_ctx.os.environ[_TARGET_PLATFORM]
    substitutions = {
        "%{gcc_path}": _GCC_PATH_MAP[target_platform],
        "%{nvcc_path}": _NVCC_PATH_MAP[target_platform],
        "%{cuda_capabilities}": _CUDA_CAPABILITIES_MAP[target_platform],
    }
    host_substitutions = {
        "%{gcc_path}": _GCC_PATH_MAP["x86_64"],
        "%{nvcc_path}": _NVCC_PATH_MAP["x86_64"],
        "%{cuda_capabilities}": _CUDA_CAPABILITIES_MAP["x86_64"],
    }
    _tpl(
        repository_ctx,
        "crosstool_wrapper_driver_is_not_gcc.py",
        host_substitutions,
        "crosstool/scripts/crosstool_wrapper_driver_is_not_gcc_host.py",
    )
    _tpl(
        repository_ctx,
        "crosstool_wrapper_driver_is_not_gcc.py",
        substitutions,
        "crosstool/scripts/crosstool_wrapper_driver_is_not_gcc.py",
    )
    _tpl(
        repository_ctx,
        "BUILD",
        substitutions,
        "crosstool/BUILD",
    )
    _tpl(
        repository_ctx,
        "cc_toolchain_config.bzl",
        substitutions,
        "crosstool/cc_toolchain_config.bzl",
    )

toolchain_configure = repository_rule(
    environ = [
        _TARGET_PLATFORM,
    ],
    implementation = _toolchain_impl,
)
