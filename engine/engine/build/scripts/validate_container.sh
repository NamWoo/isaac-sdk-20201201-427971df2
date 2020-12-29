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

# script validate a container against RACI

# fail on error

set -o errexit -o pipefail -o noclobber -o nounset

# default values

HOST="rpt-dgxstation.nvidia.com"
REPOSITORY="nvstaging/isaac-validation"
TOKEN=""
EVALUATOR=""

function show_help {
    echo "Usage: $0 [ -H <host> ] [ -r <reposistory> ] -t <token> -e <evaluator> <container-name> [ <container-name> ... ]"

    echo "Arguments:"
    echo "  -r <repository>     Repository the container was uploaded to"
    echo "  -H <host>           RACI host, default ${HOST}"
    echo "  -t <token>          RACI API token"
    echo "  -e <evaluator>      Name of evaluator to run against the container(s)"
    echo "  <container-name>    Tag of the container to validate"

    exit 1
}

function print_log {
    local EVALUATOR_ID="${1}"

    local REQUEST_URL="https://${HOST}:276/raci/v1/EvaluatorTestGetLogFileByName?EvaluatorId=${EVALUATOR_ID}&LogFileName=console_.txt"

    RESULT=$(curl -sS -k -X POST -H "Authorization: Bearer ${TOKEN}" "${REQUEST_URL}")

    TEXT=$(echo "${RESULT}" | jq -r '.fileText')

    echo "${TEXT}"
}

# get command line arguments
OPTIONS=r:H:t:e:h
LONGOPTS=repository:,host:,token:,evaluator:,help
PARSED=$(getopt --options="${OPTIONS}" --longoptions="${LONGOPTS}" --name "$0" -- "$@")
eval set -- "${PARSED}"

while true
do
    case "$1" in
        -r|--repository)
            REPOSITORY="$2"
            shift 2
            ;;
        -t|--token)
            TOKEN="$2"
            shift 2
            ;;
        -H|--host)
            HOST="$2"
            shift 2
            ;;
        -e|--evaluator)
            EVALUATOR="$2"
            shift 2
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

CONTAINERS=("$@")

if [ ${#CONTAINERS[@]} -eq 0 -o -z "${EVALUATOR}" ]
then
    show_help
    exit 1
fi

JOBS=()

# kick off validation one by one

for CONTAINER in "${CONTAINERS[@]}"
do
    REMOTETAG="nvcr.io/${REPOSITORY}/${CONTAINER}"
    OUTPUTFOLDER="${CONTAINER}_$(hostname)_$(date +'%Y%m%d%H%M%S.%N')"

    if [ ! -z ${BUILD_NUMBER:-} ]
    then
        OUTPUTFOLDER+="_${BUILD_NUMBER}"
    fi

    echo "Submitting job for container ${REMOTETAG}..."

    REQUEST_URL="https://${HOST}:276/raci/v1/EvaluatorRun?EvaluatorName=${EVALUATOR}&ResultDir=/tmp/${OUTPUTFOLDER}&ContainerName=${REMOTETAG}"

    echo "Submitting to ${REQUEST_URL}..."

    RESULT=$(curl -sS -k -X POST -H "Authorization: Bearer ${TOKEN}" "${REQUEST_URL}")
    ID=$(echo "${RESULT}" | jq -r '.id')
    RUNSTATUS=$(echo "${RESULT}" | jq -r '.runStatus')

    if [ "${RUNSTATUS}" != "ok" ]
    then
        echo "Failed to submit container ${REMOTETAG}, got status '${RUNSTATUS}' for job id '${ID}'"

        ERRORCODE=$(echo "${RESULT}" | jq -r '.error.code')
        ERRORMESSAGE=$(echo "${RESULT}" | jq -r '.error.message')

        if [ -n "${ERRORCODE}" -a "${ERRORCODE}" != "(null)" -a -n "${ERRORMESSAGE}" ]
        then
            echo "Error ${ERRORCODE}: ${ERRORMESSAGE}"
        else
            echo "No other error information was provided. Full response:"
            echo "${RESULT}"
        fi

        print_log "${ID}"

        exit 1
    fi

    echo "Job submitted, got job ID ${ID}"

    JOBS+="${ID}"
done

echo "Done submitting jobs, waiting for results..."

for JOB in "${JOBS[@]}"
do
    print_log "${JOB}"

    RESULT=$(curl -sS -k -X GET -H "Authorization: Bearer ${TOKEN}" "https://${HOST}:276/raci/v1/EvaluatorTestInfo/${JOB}")

    RUNSTATUS=$(echo "${RESULT}" | jq -r '.runStatus')

    if [ "${RUNSTATUS}" != "ok" ]
    then
        echo "Evaluation of job id '${ID}' failed, got status '${RUNSTATUS}' for job id '${ID}'"
        exit 1
    else
        echo "${JOB} finished successfully."
    fi
done

echo "All done."
