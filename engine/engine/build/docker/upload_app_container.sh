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

# script to upload a single app container to NCG

# fail on error

set -o errexit -o pipefail -o noclobber -o nounset

# default values

# repository on nvcr to use
REPOSITORY="nvstaging/isaac-validation"

# whether to keep the container locally after pushing
KEEP="false"

# username for login
USER='$oauthtoken'

# password for login
PASSWORD=''

# additional tag to apply when uploading, default to keep the tag given in the name
TAG_TO_APPLY=''

# whether to pull the container from the registry, used for re-tagging
PULL="false"

function show_help {
    echo "Usage: $0 [ -r <repository> ] [ -u <user> ] [ -p <password> ] [ -t <tag> ] [ -k ] [ -P ] <container-name> [ <container-name> ... ]"

    echo "Arguments:"
    echo "  -r <repository>     Repository to upload to, default: '${REPOSITORY}'"
    echo "  -k                  Keep the container locally (default: trash the container after upload)"
    echo "  -t <tag>            Tag to add to the container (ex. 'latest')"
    echo "  -u <user>           User name for registry login"
    echo "  -p <password>       Password for registry login"
    echo "  -P                  Pull before uploading, useful for retagging"
    echo "  <container-name>    Tag of the container to upload"

    exit 1
}

# get command line arguments
OPTIONS=r:khu:p:t:P
LONGOPTS=repository:,keep,help,user:,password:,tag:,pull
PARSED=$(getopt --options="${OPTIONS}" --longoptions="${LONGOPTS}" --name "$0" -- "$@")
eval set -- "${PARSED}"

while true
do
    case "$1" in
        -r|--repository)
            REPOSITORY="$2"
            shift 2
            ;;
        -u|--user)
            USER="$2"
            shift 2
            ;;
        -p|--password)
            PASSWORD="$2"
            shift 2
            ;;
        -k|--keep)
            KEEP="true"
            shift
            ;;
        -t|--tag)
            TAG_TO_APPLY="$2"
            shift 2
            ;;
        -P|--pull)
            PULL="true"
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

# Make sure we log in

echo "$PASSWORD" | docker login -u "${USER}" --password-stdin nvcr.io

# Upload containers one at a time

CONTAINERS=("$@")

for CONTAINER in "${CONTAINERS[@]}"
do
    CONTAINERBASE="${CONTAINER%:*}"

    REMOTETAG="nvcr.io/${REPOSITORY}/${CONTAINER}"

    if [ "${PULL}" = "true" ]
    then
        docker pull "${REMOTETAG}"
    else
        docker tag "${CONTAINER}" "${REMOTETAG}"
        docker push "${REMOTETAG}"
    fi

    if [ -n "${TAG_TO_APPLY}" ]
    then
        WITH_TAG="nvcr.io/${REPOSITORY}/${CONTAINERBASE}:${TAG_TO_APPLY}"

        docker tag "${REMOTETAG}" "${WITH_TAG}"
        docker push "${WITH_TAG}"
    fi

    if [ "${KEEP}" = "false" ]
    then
        if [ "${PULL}" = "false" ]
        then
            docker rmi "${CONTAINER}"
        fi

        docker rmi "${REMOTETAG}"

        if [ -n "${TAG_TO_APPLY}" ]
        then
            docker rmi "${WITH_TAG}"
        fi
    fi
done
