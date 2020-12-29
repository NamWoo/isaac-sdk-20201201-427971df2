"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def isaac_http_archive(licenses, name, **kwargs):
    """
    An Isaac HTTP third party archive. Augment the standard Bazel HTTP archive workspace rule.
    Mandatory licenses label.
    """
    maybe(
        repo_rule = http_archive,
        name = name,
        **kwargs
    )

def isaac_new_git_repository(licenses, name, **kwargs):
    """
    An Isaac Git third party repository. Augment the standard new Bazel Git repository workspace
    rule. Mandatory licenses label.
    """
    maybe(
        repo_rule = new_git_repository,
        name = name,
        **kwargs
    )

def isaac_git_repository(licenses, name, **kwargs):
    """
    An Isaac Git third party repository. Augment the standard Bazel Git repository workspace rule.
    Mandatory licenses label.
    """
    maybe(
        repo_rule = git_repository,
        name = name,
        **kwargs
    )

def isaac_new_local_repository(licenses, name, **kwargs):
    """
    An Isaac local third party repository. Augment the standard Bazel Git repository workspace rule.
    Mandatory licenses label.
    """
    maybe(
        repo_rule = native.new_local_repository,
        name = name,
        **kwargs
    )
