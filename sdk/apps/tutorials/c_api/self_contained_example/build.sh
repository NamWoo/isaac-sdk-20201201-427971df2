#!/usr/bin/env bash

# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Script to build the C-API stand alone example application
# usage: copy the sample application files into your deployed C-API folder,
#        then from the deployed C-API folder run ./build.sh

# if c_api_example exists, remove it
if [ -f "./c_api_example" ]; then
  rm ./c_api_example
fi

# Checks for current platform
PLATFORM_INFO=$(uname -a)
PLATFORM="_x86_64"
if [[ "$PLATFORM_INFO" == *"aarch64"* ]]; then
  PLATFORM="_jetpack44"
fi

# Checks if need to link against platform-specific prebuilt module
SO=$(find './packages/engine_tcp_udp' -name '*.so')
if [ ! -z $SO ]; then
  PLATFORM=''
fi

# compile c_api_example
gcc c_api_example.c \
    -L ./packages/engine_c_api -lisaac_c_api \
    -L ./packages/sight -lsight_module \
    -L ./packages${PLATFORM}/engine_tcp_udp -lengine_tcp_udp_module \
    -o c_api_example

# if c_api_example exists, run it
if [ -f "./c_api_example" ]; then
  # export library paths
  export LD_LIBRARY_PATH=./packages/sight:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=./packages/engine_c_api:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=./packages${PLATFORM}/engine_tcp_udp:$LD_LIBRARY_PATH

  # run the example
  ./c_api_example
fi
