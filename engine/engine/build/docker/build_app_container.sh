#!/bin/bash

#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# script to build a single app, and wrap it in a docker container

# fail on error

set -o errexit -o pipefail -o noclobber -o nounset

# this is empty initially, defined here so the hook can catch it

TMPDIR=""
TMPFILE=""

# path to this script

SCRIPT_PATH=$(dirname "$(readlink -f "$0")")

#### default arguments

# target platform to build for
PLATFORM="x86_64"

# location of the remote bazel cache to use, empty for no cache
CACHE=""

# only output the image name and exit, can be used to query the name in other scripts
IMAGE_NAME_ONLY=false

# whether to enable verbose output
VERBOSE=false

function verbose_echo
{
    if [ "${VERBOSE}" = true ]
    then
        echo "$@"
    fi
}

# exit hook

function clean_on_exit
{
    if [ "${TMPDIR}" -a -d "${TMPDIR}" ]
    then
        verbose_echo "${FUNCNAME[0]}: Removing ${TMPDIR}..."
        rm -rf "${TMPDIR}"
    fi

    if [ "${TMPFILE}" -a -f "${TMPFILE}" ]
    then
        verbose_echo "${FUNCNAME[0]}: Removing ${TMPFILE}..."
        rm -f "${TMPFILE}"
    fi

    popd > /dev/null 2>&1
}

trap clean_on_exit EXIT

function show_help
{
    echo "Usage: $0 [ -p <platform> ] [ -h ] [ -- ] <app> [ <app> ... ]"

    echo "Arguments:"
    echo "  -p <platform>    The platform to build for (valid choices: x86_64 (default), jetpack44)"
    echo "  -c <cache>       Location of remote bazel cache, empty for no cache (default: empty)"
    echo "  -h               Show this help"
    echo "  -i               Only display the name of the docker image without building"
    echo "  --               Indicates end of argument list"
    echo "  <app>            Name of the app(s) to build (required), example: //apps/carter:carter"
}

function go_to_workspace_root
{
    local WORKSPACE_LOCATION="${SCRIPT_PATH}"

    while [ ! -f "${WORKSPACE_LOCATION}/WORKSPACE" -a $(readlink -f "${WORKSPACE_LOCATION}") != "/" ]
    do
        WORKSPACE_LOCATION="${WORKSPACE_LOCATION}/.."
    done

    if [ ! -f "${WORKSPACE_LOCATION}/WORKSPACE" ]
    then
        echo "Could not locate WORKSPACE file, please call this script from within repository"
        exit 1
    fi

    verbose_echo "Found WORKSPACE at $(readlink -f ${WORKSPACE_LOCATION})"
    pushd "${WORKSPACE_LOCATION}/.." > /dev/null
}

# build a docker image for a single app

function build_single_app
{
    local APP="${1}"
    local PLATFORM="${2}"
    local TAG=$(get_image_name "${APP}" "${PLATFORM}")
    local HOMEDIR="${HOME}"

    TMPDIR=$(mktemp -d)
    TMPFILE=$(mktemp)

    local CACHEARG=""
    local CACHEMOUNT="-v ${HOME}/.cache/bazel:${HOME}/.cache/bazel"

    # if a remote cache was specified, then skip mounting the local cache, and pass through the
    # remote cache
    if [ -n "${CACHE}" ]
    then
        CACHEARG="-c ${CACHE}"
        CACHEMOUNT=""
        HOMEDIR="${TMPDIR}"
    fi

    local OUTPUTDIR="${TMPDIR}/app"

    docker run \
           -e LOGNAME="$(id -u)" \
           -e USER="$(id -u)" \
           -e HOME="${HOMEDIR}" \
           -u="$(id -u)" \
           -v "${PWD}":/src/workspace \
           -v /etc/passwd:/etc/passwd \
           -v "${TMPDIR}":"${TMPDIR}" \
           ${CACHEMOUNT} \
           -w /src/workspace/sdk \
           isaacbuild \
           ../engine/engine/build/deploy.sh ${CACHEARG} -p "${APP}-pkg" -d "${PLATFORM}" -h localhost --deploy_path "${TMPDIR}/app"

    build_docker_file "${TMPFILE}" "${TMPDIR}" "${PLATFORM}"

    docker build -t "${TAG}" -f "${TMPFILE}" "${TMPDIR}"

    rm -f "${TMPFILE}"
}

# create the per-app dockerfile

function build_docker_file
{
    local OUTPUT="${1}"
    local DIRECTORY="${2}"
    local PLATFORM="${3}"

    case "${PLATFORM}" in
        "x86_64")
            build_docker_file_x86 "${OUTPUT}"
            ;;
        "jetpack44")
            build_docker_file_jetpack "${OUTPUT}" "${DIRECTORY}"
            ;;
        *)
            echo "Unsupported platform: ${PLATFORM}"
            exit 1
    esac
}

function build_docker_file_x86
{
    local OUTPUT="${1}"
    local RACI_DEPS="pymeasure==1.6.8"

    cat >> "${OUTPUT}" <<EOF
FROM isaacbuild

COPY app/* /app/

WORKDIR /app
RUN python3 -m pip install "${RACI_DEPS}" --verbose || exit 1

EOF
}

function build_docker_file_jetpack
{
    local OUTPUT="${1}"
    local DIRECTORY="${2}"

    cp engine/engine/build/scripts/install_dependencies_jetson.sh "${DIRECTORY}"
    cp engine/engine/build/scripts/requirements_jetson.txt "${DIRECTORY}"

    cat >> "${OUTPUT}" <<EOF
FROM nvcr.io/nvidia/l4t-base:r32.4.3

COPY install_dependencies_jetson.sh /
COPY requirements_jetson.txt /

RUN /install_dependencies_jetson.sh --local

COPY app/* /app/

WORKDIR /app

EOF
}

# clean the target name from incompatible characters, and tag it with the current git hash

function get_image_name
{
    local APP="${1}"
    local PLATFORM="${2}"
    local GIT_HASH=$(git rev-parse --short HEAD)

    local CLEANED_APP=$(echo "${APP}" | tr '/:' '__' | sed 's/^__//')

    echo "${CLEANED_APP}-${PLATFORM}:${GIT_HASH}"
}

# refer to other script to create parent container for now

function build_dev_container
{
    pushd engine

    ./engine/build/docker/create_image.sh

    popd
}

# get command line arguments
OPTIONS=p:c:ivh
LONGOPTS=platform:,cache:imagename,verbose,help
PARSED=$(getopt --options="${OPTIONS}" --longoptions="${LONGOPTS}" --name "$0" -- "$@")
eval set -- "${PARSED}"

while true
do
    case "$1" in
        -p|--platform)
            PLATFORM="$2"
            shift 2
            ;;
        -c|--cache)
            CACHE="$2"
            shift 2
            ;;
        -i|--image_name)
            IMAGE_NAME_ONLY=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        --)
            shift
            break
            ;;
        *)
            echo "$0: Unknown argument: $1"
            exit 1
            ;;
    esac
done

# get command to run
if [[ $# -lt 1 ]]
then
    show_help
    exit 1
fi

# make sure our pathes work

go_to_workspace_root

# build dev environment container first

if [ "${IMAGE_NAME_ONLY}" = false ]
then
    build_dev_container
fi

# then build the apps one by one

APPS=("$@")

for APP in "${APPS[@]}"
do
    SHORTAPP=$(echo "${APP}" | sed 's/-pkg$//')

    if [ "${IMAGE_NAME_ONLY}" = true ]
    then
        echo $(get_image_name "${SHORTAPP}" "${PLATFORM}")
    else
        build_single_app "${SHORTAPP}" "${PLATFORM}"
    fi
done
