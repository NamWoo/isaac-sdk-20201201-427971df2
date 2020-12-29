#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# script settings
set -e  # fail on errors
set -o pipefail  # handle errors on pipes

# constants
REQUESTED_UBUNTU_VERSION="18.04"
REQUESTED_BAZEL_VERSION="3.1.0"
LOCAL_DEVICE="x86_64"
COLOR_RED='\033[0;31m'
COLOR_GREEN='\033[0;32m'
COLOR_NONE='\033[0m'
COLOR_YELLOW='\033[0;33m'

# Helper functions to be used in this script.
# Prints the error message and exits the script.
error_and_exit() {
  printf "${COLOR_RED}Error: $1${COLOR_NONE}\n"
  printf "  see: $0 --help\n"
  exit 1
}
# Prints the warning message.
warn() {
  printf "${COLOR_YELLOW}Warning: $1${COLOR_NONE}\n"
}
# Prints help message.
print_help_message() {
  printf "Usage: $0 -p <package> -h <IP> -d <device> [options]\n"
  printf "  -d|--device:       Desired target device.\n"
  printf "  -h|--host:         Host IP address.\n"
  printf "  -p|--package:      Package to deploy, for example: //foo/bar:tar.\n"
  printf "  -r|--run:          Run on remote.\n"
  printf "  -s|--symbols:      Preserve symbols when building.\n"
  printf "  -u|--user:         Local username, defaults to ${USER}.\n"
  printf "  --update:          Only deploy files that have changed.\n"
  printf "  --remote_user:     Username on target device.\n"
  printf "  --deploy_path:     Destination on target device.\n"
  printf "  --help:            Display this message.\n"
}

# Check Ubuntu and bazel version before building.
# Theses checks should reduce the number of problems reported on forums.
UBUNTU_VERSION=$(lsb_release -rs)
BAZEL_VERSION=$(bazel version | grep 'Build label' | sed 's/Build label: //')
if [[ $UBUNTU_VERSION != $REQUESTED_UBUNTU_VERSION ]]; then
  error_and_exit \
    "Isaac currently only supports Ubuntu $REQUESTED_UBUNTU_VERSION. Your Ubuntu version is $UBUNTU_VERSION."
fi
if [[ $BAZEL_VERSION != $REQUESTED_BAZEL_VERSION ]]; then
  error_and_exit \
    "Isaac requires bazel version $REQUESTED_BAZEL_VERSION. Please verify your bazel with 'bazel version' command."
fi

# Save this script's directory in case it is being called from an external workspace
dir_this_script=$(dirname "$0")

# used arguments with default values
UNAME=$USER
REMOTE_USER=nvidia
REMOTE_USER_SET=false
CACHE_SERVER_ARG=""

# print help and exit if no command line arguments
if [ $# -eq 0 ]; then
  print_help_message
  exit 0
fi

# get command line arguments
while [ $# -gt 0 ]; do
  case "$1" in
    -p|--package)
      PACKAGE="$2"
      ;;
    -d|--device)
      DEVICE="$2"
      ;;
    -c|--cache)
      CACHE_SERVER_ARG="--remote_http_cache=$2"
      ;;
    -h|--host)
      HOST="$2"
      ;;
    -u|--user)
      UNAME="$2"
      ;;
    -s|--symbols)
      NEED_SYMBOLS="True"
      shift
      continue
      ;;
    -r|--run)
      NEED_RUN="True"
      shift
      continue
      ;;
    --remote_user)
      REMOTE_USER="$2"
      REMOTE_USER_SET=true
      ;;
    --deploy_path)
      DEPLOY_PATH="$2"
      ;;
    --update)
      NEED_UPDATE="True"
      shift
      continue
      ;;
    --help)
      print_help_message
      exit 0
      ;;
    *)
      error_and_exit "Error: Invalid arguments: ${1} ${2}"
  esac
  shift
  shift
done

if [ -z "$PACKAGE" ]; then
  error_and_exit "Package must be specified with -p //foo/bar:tar."
fi
if [[ $PACKAGE != //* ]]; then
  error_and_exit "Package must start with //. For example: //foo/bar:tar."
fi

if [ -z "$HOST" ]; then
  error_and_exit "Host IP must be specified with -h IP."
fi

if [ -z "$DEVICE" ]; then
  error_and_exit "Desired target device must be specified with -d DEVICE. Valid choices: 'jetpack44', 'x86_64'."
fi

#Check if we need ssh to deploy. Potentially overwrite REMOTE_USER.
SSH_NEEDED=true
if [[ $HOST == "localhost" || $HOST == "127.0.0.1" ]]; then
  # Check username
  if [[ $REMOTE_USER_SET == false ]]; then
    # If user has not explicitly set REMOTE_USER, set it to $USER
    echo "No remote user is specified. Using '$USER' for local deployment."
    REMOTE_USER=$USER
  elif [[ $REMOTE_USER != $USER ]]; then
    warn "This is a local deployment, but remote user is explicitly specified as '$REMOTE_USER'"
  fi
  if [[ $REMOTE_USER == $USER ]]; then
    SSH_NEEDED=false
  fi
  if [[ $DEVICE != $LOCAL_DEVICE ]]; then
    warn "Deploying a '$DEVICE' package to localhost"
  fi
fi

# Split the target of the form //foo/bar:tar into "//foo/bar" and "tar"
targetSplitted=(${PACKAGE//:/ })
if [[ ${#targetSplitted[@]} != 2 ]]; then
  error_and_exit "Package '$PACKAGE' must have the form //foo/bar:tar"
fi
PREFIX=${targetSplitted[0]:2}
TARGET=${targetSplitted[1]}

TARPATH=$(bazel aquery --config $DEVICE $PREFIX:$TARGET --output=jsonproto | jq -r '
            .artifacts |
            .[] |
            select( .execPath | endswith(".tar") or endswith(".tar.gz")) |
            .execPath')
TARFILE=$(basename ${TARPATH})

echo "================================================================================"
echo "Building Minidump tools"
echo "================================================================================"
source $dir_this_script/scripts/prepare_minidump_tools.sh && wait

# build the bazel package
echo "================================================================================"
echo "Building //$PREFIX:$TARGET for target platform '$DEVICE'"
echo "================================================================================"
bazel build $CACHE_SERVER_ARG --config $DEVICE $PREFIX:$TARGET --strip=always || exit 1

# Print a message with the information we gathered so far
echo "================================================================================"
echo "Deploying //$PREFIX:$TARGET ($EX) to $REMOTE_USER@$HOST under name '$UNAME'"
echo "================================================================================"

# unpack the package in the local tmp folder
rm -f /tmp/$TARFILE
cp $TARPATH /tmp/
rm -rf /tmp/$TARGET
mkdir /tmp/$TARGET
tar -xf /tmp/$TARFILE -C /tmp/$TARGET

# Deploy directory
if [ -z "$DEPLOY_PATH" ]
then
  DEPLOY_PATH="/home/$REMOTE_USER/deploy/$UNAME/"
fi

# sync the package folder to the remote
REMOTE_USER_AND_HOST="$REMOTE_USER@$HOST:"
# Special case: Don't ssh if not needed
if [[ $SSH_NEEDED == false ]]; then
  REMOTE_USER_AND_HOST=""
fi
# Special case: don't delete existing folder, don't update newer destination files.
if [[ ! -z $NEED_UPDATE ]]; then
  RSYNC_POLICY="--update"
else
  RSYNC_POLICY="--delete"
fi

rsync -avz $RSYNC_POLICY --checksum --rsync-path="mkdir -p $DEPLOY_PATH/ && rsync" \
    /tmp/$TARGET $REMOTE_USER_AND_HOST$DEPLOY_PATH
status=$?
if [ $status != 0 ]; then
  error_and_exit "rsync failed with exit code $status"
fi

if [[ -z $NEED_SYMBOLS ]]; then
  echo "================================================================================"
  echo "To grab symbols pass -s/--symbols"
  echo "================================================================================"
else
  echo "================================================================================"
  echo "Grabbing symbols"
  echo "================================================================================"
  # Retain symbols in all binaries
  bazel build $CACHE_SERVER_ARG --config $DEVICE $PREFIX:$TARGET --strip=never || exit 1

  # Unpack the package in the local tmp folder
  rm -f /tmp/$TARFILE
  cp $TARPATH /tmp/
  rm -rf /tmp/$TARGET
  mkdir /tmp/$TARGET
  tar -xf /tmp/$TARFILE -C /tmp/$TARGET

  EXECUTABLES=$(find /tmp/$TARGET -executable -type f)
  for exe in $EXECUTABLES
  do
    if [[ ! -z $(file "$exe" | sed '/ELF/!d') ]]; then
      $dir_this_script/scripts/process_syms.sh $exe
    fi
  done
  wait
fi
echo "================================================================================"
printf "${COLOR_GREEN}Deployed${COLOR_NONE}\n"
echo "================================================================================"

if [[ ! -z $NEED_RUN ]]; then
  echo "================================================================================"
  echo "Running on Remote"
  echo "================================================================================"
  # echo "cd $DEPLOY_PATH/$TARGET; ./$PREFIX/${TARGET::-4}"
  ssh -t $REMOTE_USER@$HOST "cd $DEPLOY_PATH/$TARGET; ./$PREFIX/${TARGET::-4}"
fi
