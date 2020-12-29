#!/bin/bash
#####################################################################################
# Copyright (c) 2019,2020, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# Create Docker image tagged "isaacbuild" for building Isaac SDK targets, useful for development.

# fail on error

set -o errexit -o pipefail -o noclobber -o nounset

# path to this script

SCRIPT_PATH=$(dirname "$(readlink -f "$0")")

pushd ${SCRIPT_PATH}

docker build -t isaacbuild -f Dockerfile ..

popd
