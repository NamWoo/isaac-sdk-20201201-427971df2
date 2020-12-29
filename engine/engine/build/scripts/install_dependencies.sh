#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# Only for debug purposes
# set -ex

# This script shall not be run with sudo
if [ "$EUID" -eq 0 ] && [ -x "$(command -v sudo)" ]; then
  echo "Please do not run as root"
  exit
fi

if ! [ -x "$(command -v sudo)" ]; then
  sudo(){
    PYTHON="python3 -m pip install"
    eval "$@"
  }
else
  PYTHON="python3 -m pip install --user"
fi

install_package(){
 echo -e "\e[32m[INFO] Installing packages - $@\e[0m"
 sudo apt-get install -y $@ -qq > /dev/null
 if [ $? -ne 0 ]; then
   echo -e "\e[31m[ERROR] Failed to install packages - $@\e[0m"
   exit 1
 fi
 echo -e "\e[32m[INFO] Successfully installed packages - $@\e[0m"
}

# Updates the list of available packages and their versions to get up-to-date packages
sudo apt-get update
# Install ffmpeg to save outliers image stream for ML model evaluation
install_package ffmpeg
# Install build and deployment tools
install_package lsb-core
# Installs C++ dev tools
install_package git git-lfs git-review build-essential g++ redis-server
git lfs install
# Installs Bazel deps
install_package pkg-config zip zlib1g-dev unzip curl
# Installs core deps
install_package uuid-dev
# Installs 3D app deps
install_package libx11-dev libxcursor-dev libxrandr-dev libxinerama-dev
# Installs audio DL training app deps
install_package sox
# Installs ARM building deps
install_package gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
# Installs codecs
install_package libvpx5 libtwolame-dev libtheora-bin libspeex1
install_package libopus0 libmp3lame0 libvdpau1 openexr libmpg123-0
# Install Sphinx deps
install_package python3-pip \
    latexmk texlive-latex-recommended texlive-latex-extra texlive-fonts-recommended \
    texlive-luatex texlive-xetex
# Install Zed Camera deps
install_package libudev-dev
# Install I2C deps
install_package libi2c-dev
# Install Realsense Camera dependencies
install_package libhidapi-libusb0 libturbojpeg
# Installs python3
install_package python3-dev libssl-dev
install_package python3-tk libopenmpi-dev
# Install GLib and GStreamer for open source GStreamer and NVIDIA DeepStream components.
install_package glib2.0 gstreamer1.0-plugins.* libgstreamer1.0.* libgstreamer-plugins-.*1.0.*
# Install jq for deploy script
install_package jq
# Install Sphinx (documentation generation)
$PYTHON --upgrade pip
$PYTHON launchpadlib
$PYTHON --upgrade setuptools

# Install packages needed for RACI Workflows
TMPFOLDER=$(mktemp -d)
OUTPUT_SH=$TMPFOLDER/setup_python.sh
curl -s -L https://packages.nvidia.com/install/repositories/dataplatform_prod/python/script.python.sh -o $OUTPUT_SH || \
                       { echo "Error:'script.python.sh' download failed"; exit 1; }
chmod +x $OUTPUT_SH || { echo "Error:'$OUTPUT_SH' permission change failed"; exit 1; }
sudo bash $OUTPUT_SH || { echo "Error:'$OUTPUT_SH' execution failed"; exit 1; }
install_package python3-scp python-scp
install_package python-elasticsearch python3-elasticsearch

# Dependency of Python onnx package
install_package protobuf-compiler

REQ_PATH=$(dirname "$(readlink -f "$0")")
$PYTHON -r ${REQ_PATH}/requirements.txt --verbose || exit 1
# Enable ipywidgets in jupyter for engine/pyalice/gui/composite_widget
# ipywidgets installation instruction: https://ipywidgets.readthedocs.io/en/latest/user_install.html
# this fixes issue https://github.com/jupyter-widgets/ipywidgets/issues/2220
jupyter nbextension enable --py widgetsnbextension

# Install/setup pyparsing for Python3.
$PYTHON pyparsing

# Installs Bazel
TMPFOLDER=$(mktemp -d)
OUTPUT_SH=$TMPFOLDER/bazel.sh
curl -s -L https://github.com/bazelbuild/bazel/releases/download/3.1.0/bazel-3.1.0-installer-linux-x86_64.sh -o $OUTPUT_SH || \
                       { echo "Error:bazel installer download failed "; exit 1; }
chmod +x $OUTPUT_SH || { echo "Error:'$OUTPUT_SH' permission change failed"; exit 1; }
sudo bash $OUTPUT_SH || { echo "Error:'$OUTPUT_SH' execution failed"; exit 1; }

echo "Installation Succeeded"
