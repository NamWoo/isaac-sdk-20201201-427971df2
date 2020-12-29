#!/bin/bash
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

if [ -f /sys/module/tegra_fuse/parameters/tegra_chip_id ]; then
    case $(cat /sys/module/tegra_fuse/parameters/tegra_chip_id) in
        25)
            TARGET_PLATFORM="JETSON Xavier"
            if [ -f /etc/nv_tegra_release ]; then
                L4T_RELEASE=$(echo $(head -n 1 /etc/nv_tegra_release) \
                    | cut -f 2 -d ' ' | grep -Po '(?<=R)[^;]+')
                L4T_REVISION=$(echo $(head -n 1 /etc/nv_tegra_release) \
                    | cut -f 2 -d ',' | grep -Po '(?<=REVISION: )[^;]+')
                L4T_VERSION="$L4T_RELEASE.$L4T_REVISION"
                case $L4T_VERSION in
                    "32.3.1")
                        JETPACK_VERSION="4.3" ;;
                    "32.4.3")
                        JETPACK_VERSION="4.4" ;;
                    "32.4.4")
                        JETPACK_VERSION="4.4.1" ;;
                    *)
                        JETPACK_VERSION="NA" ;;
                esac
            else
                JETPACK_VERSION="NA"
            fi ;;
        *)
            TARGET_PLATFORM="NA"
            JETPACK_VERSION="NA" ;;
    esac
else
    TARGET_PLATFORM="x86"
    JETPACK_VERSION="NA"
fi

# Export variables
export TARGET_PLATFORM
export JETPACK_VERSION
