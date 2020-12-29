#####################################################################################
# Copyright (c) 2019,2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# Full Isaac SDK build environment for development activities.

# this is the latest known good version from the 10.2-devel-ubuntu18.04 tag
FROM nvidia/cudagl@sha256:9f6be490b5e1571927ee56f0dd5eadbe947f0bf2fa85f474887126553a0d1bbf

# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive

# weget: for vulkan and cudnn download
# apt-utils: https://github.com/phusion/baseimage-docker/issues/319
# aha, rsync, xz-utils: Install AHA utility to prepare HTML reports from the test results
RUN apt-get update && apt-get install -y --no-install-recommends wget apt-utils aha rsync xz-utils lsb-release

# Install Vulkan SDK, see https://gitlab.com/mlamarre/vulkan/blob/ubuntu18.04/Dockerfile
ENV NVIDIA_DRIVER_CAPABILITIES compute,graphics,utility,display,video
RUN wget -qO - http://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - && \
  wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.1.121-bionic.list http://packages.lunarg.com/vulkan/1.1.121/lunarg-vulkan-1.1.121-bionic.list && \
  apt update && apt install -y vulkan-sdk && apt upgrade -y && apt autoremove -y
COPY docker/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json

# Install cuDNN v8 as described in https://hub.docker.com/r/kaixhin/cudnn/dockerfile
ENV CUDNN_RUN_PKG=libcudnn8_8.0.3.33-1+cuda10.2_amd64.deb
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/$CUDNN_RUN_PKG && \
  dpkg -i $CUDNN_RUN_PKG

ENV CUDNN_DEV_PKG=libcudnn8-dev_8.0.3.33-1+cuda10.2_amd64.deb
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/$CUDNN_DEV_PKG && \
  dpkg -i $CUDNN_DEV_PKG

# Configure the build for CUDA configuration
ENV LD_LIBRARY_PATH /usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH

# Install SDK dependencies
ADD scripts/install_dependencies.sh engine/build/scripts/
ADD scripts/requirements.txt engine/build/scripts/
RUN apt update && engine/build/scripts/install_dependencies.sh

COPY docker/docker_entrypoint.sh /
ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["/bin/bash"]
