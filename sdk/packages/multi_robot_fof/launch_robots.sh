#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# Defaults
TRANSPORTERS=3
STATIONS=4

# Get command line arguments
while [ $# -gt 0 ]; do
  case "$1" in
    -t|--transporters)
      TRANSPORTERS="$2"
      ;;
    -s|--stations)
      STATIONS="$2"
      ;;
    *)
      error_and_exit "Error: Invalid arguments: ${1} ${2}"
  esac
  shift
  shift
done


# Base values; change if the starting ports should change
SIM_OUTPUT_PORT_TRANSPORTER_BASE=45000
SIM_OUTPUT_PORT_STATION_BASE=46000
SIGHT_PORT_BASE=4000

COUNTER=0
PIDs=()

# Run build-graph before launching transporters
if [ "${TRANSPORTERS}" -gt 0 ]; then
  bazel run //packages/multi_robot_fof:build_graph
fi

# For each index from 0 to given transporter amount
while [ "${COUNTER}" -lt "${TRANSPORTERS}" ]; do
    # Calculate effective port numbers
    SIM_OUTPUT_PORT=`echo "${SIM_OUTPUT_PORT_TRANSPORTER_BASE}+(${COUNTER}*2)" | bc`
    SIGHT_PORT=`echo "${SIGHT_PORT_BASE}+${COUNTER}" | bc`

    # Run app
    bazel run //packages/multi_robot_fof:transporter --\
          --sim_output_port ${SIM_OUTPUT_PORT}\
          --sight_port ${SIGHT_PORT}\
          --robot_index ${COUNTER}&
    # Take note of PID for killing it later
    PIDs+=($!)

    let COUNTER=COUNTER+1
    # Only sleep if this is not the last instance to run
    if [ "${COUNTER}" -lt "${TRANSPORTERS}" ]; then
        sleep 3
    fi
done

COUNTER=0

# For each index from 0 to given station amount
while [ "${COUNTER}" -lt "${STATIONS}" ]; do
    OFFSET_COUNTER=`echo ${COUNTER}+${TRANSPORTERS} | bc`
    # Calculate effective port numbers
    SIM_OUTPUT_PORT=`echo "${SIM_OUTPUT_PORT_STATION_BASE}+(${COUNTER}*2)" | bc`
    SIGHT_PORT=`echo "${SIGHT_PORT_BASE}+${OFFSET_COUNTER}" | bc`

    # Run app
    bazel run //packages/multi_robot_fof:station --\
          --sim_output_port ${SIM_OUTPUT_PORT}\
          --sight_port ${SIGHT_PORT}\
          --robot_index ${COUNTER}&
    # Take note of PID for killing it later
    PIDs+=($!)

    let COUNTER=COUNTER+1
    # Only sleep if this is not the last instance to run
    if [ "${COUNTER}" -lt "${STATIONS}" ]; then
        sleep 3
    fi
done

# Wait for CTRL+C
( trap exit SIGINT ; read -r -d '' _ </dev/tty )
# Kill all PIDs started above
kill -s SIGKILL ${PIDs[@]}
