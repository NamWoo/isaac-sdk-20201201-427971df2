#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# Example script to build all the Isaac framework targets, using Bazel cache in a virtual Docker
# environment using the "isaacbuild" Docker image (prerequisite).

OUTPUT_DIR=/tmp/build_output
if [ ! -d "$OUTPUT_DIR" ]; then
    mkdir "$OUTPUT_DIR"
fi

docker run \
    -e LOGNAME="$(id -u)" \
    -e USER="$(id -u)" \
    -u="$(id -u)" \
    -v "$PWD":/src/workspace \
    -v /etc/passwd:/etc/passwd \
    -v /tmp/build_output:/tmp/build_output \
    -w /src/workspace \
    isaacbuild \
    bazel --output_user_root=/tmp/build_output build --config=jetpack44 ... &&
docker run \
    -e LOGNAME="$(id -u)" \
    -e USER="$(id -u)" \
    -u="$(id -u)" \
    -v "$PWD":/src/workspace \
    -v /etc/passwd:/etc/passwd \
    -v /tmp/build_output:/tmp/build_output \
    -w /src/workspace \
    isaacbuild \
    bazel --output_user_root=/tmp/build_output build --config=x86_64 ...
