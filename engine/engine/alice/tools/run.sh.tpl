#!/bin/sh
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
EXECUTABLE=engine/alice/tools/main
if [ -f $EXECUTABLE ]; then
  $EXECUTABLE --app {APP_JSON_FILE} $@
elif [ -f "../isaac/"$EXECUTABLE ]; then
  ../isaac/$EXECUTABLE --app {APP_JSON_FILE} $@
elif [ -f "external/com_nvidia_isaac_engine/"$EXECUTABLE ]; then
  external/com_nvidia_isaac_engine/$EXECUTABLE --app {APP_JSON_FILE} $@
else
  echo "Could not find proper executable"
  exit 1
fi
