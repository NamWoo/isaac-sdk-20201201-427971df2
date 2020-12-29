#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

set -o errexit -o pipefail -o noclobber -o nounset

UNAME=nvidia

# find out location of script to we can access local files
SCRIPT_LOCATION=$(dirname $(readlink -f $0))

### default arguments

# whether this is a local installation, i.e. it is executed directly on the device, or in a
# cross-platform docker container, default is to ssh to a remote host
LOCAL_INSTALL=false

# for remote install, host name or IP to connect to
HOST=""

# get command line arguments
while [ $# -gt 0 ]
do
  case "$1" in
    -h|--host)
      HOST="$2"
      shift 2
      ;;
    -l|--local)
      LOCAL_INSTALL=true
      shift
      ;;
    -u|--user)
      UNAME="$2"
      shift 2
      ;;
    *)
      printf "Error: Invalid arguments: %1 %2\n"
      exit 1
  esac
done

if [ -z "${HOST}" -a "${LOCAL_INSTALL}" = false ]
then
  echo "Error: Jetson device IP must be specified with -h IP."
  exit 1
fi

# This function will be ran on the target device
remote_function() {
  local IS_LOCAL="$1"

  # no sudo available in Docker
  if [ "${IS_LOCAL}" = true ]
  then
      SUDO=""
  else
      SUDO="sudo"
  fi

  # Install packages
  ${SUDO} apt update && ${SUDO} apt install -y rsync curl libhidapi-libusb0 \
       libturbojpeg python3-pip redis-server

  # Packages for v4l2 support
  ${SUDO} apt install -y libv4l-dev

  # Packages for torch
  ${SUDO} apt install -y libopenblas-dev libopenmpi-dev

  # packages needed for jupyter notebooks
  ${SUDO} apt install -y libjpeg-dev libfreetype6-dev libffi-dev

  # numerical routines for numpy
  ${SUDO} apt install -y liblapack3 libblas3 libatlas-base-dev

  # indirect dependency of numpy-quaternion
  ${SUDO} apt install -y llvm-10 gfortran
  ${SUDO} update-alternatives --install /usr/bin/llvm-config llvm-config /usr/bin/llvm-config-10 10

  # for remote install, we don't get the script dir, so use relative path
  if [ "${IS_LOCAL}" = false ]
  then
      REQUIREMENTS_FILE=requirements_jetson.txt
  else
      REQUIREMENTS_FILE=${SCRIPT_LOCATION}/requirements_jetson.txt
  fi

  # do not use `pip install -r` here because it doesn't handle inter-dependencies
  # properly
  for REQUIREMENT in $(cat ${REQUIREMENTS_FILE} | grep -v '^#' | grep -v '^[[:space:]]*$')
  do
      python3 -m pip install --user ${REQUIREMENT}
  done

  # Query path to local install so we can find our binary
  local PYTHON_PATH=$(python3 -c 'import site; print(site.USER_BASE)')

  ${PYTHON_PATH}/bin/jupyter nbextension enable --py widgetsnbextension

  if [ "${IS_LOCAL}" = false ]
  then
    # Give user permission to use i2c and tty devices
    ${SUDO} usermod -a -G i2c,dialout $USER

    # Blacklist nvs_bmi160 kernel mod
    echo blacklist nvs_bmi160 | ${SUDO} tee /etc/modprobe.d/blacklist-nvs_bmi160.conf > /dev/null
  fi
}

if [ "${LOCAL_INSTALL}" = true ]
then
  remote_function "${LOCAL_INSTALL}"
else
  # Installs dependencies on Jetson devices
  scp ${SCRIPT_LOCATION}/requirements_jetson.txt $UNAME@$HOST:
  ssh -t $UNAME@$HOST "$(declare -pf remote_function); remote_function false"

  ssh -t $UNAME@$HOST "sudo shutdown -r 1"

  echo "Rebooting the Jetson device in 1 minute"
fi

