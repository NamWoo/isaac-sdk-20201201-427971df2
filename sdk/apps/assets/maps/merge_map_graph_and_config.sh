#!/bin/bash
#####################################################################################
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
# This script merges graph.json and config.json files into a single json file

set -e

if [ "$#" -eq 0 ]; then
    echo Provide the graph file paths as input.
    echo Example usage: ./apps/assets/maps/merge_map_graph_and_config.sh ./apps/assets/maps/*graph.json
    exit 1
fi

echo Arguments: "$@"
echo ==========

for graph_file in "$@"
do
  if [ ! -f $graph_file ]
  then
    echo "$graph_file does not exist."
    exit 1
  fi
  if [[ "$graph_file" != *graph.json ]]
  then
    echo $graph_file does not end with "graph.json"
    exit 1
  fi
  base_name="${graph_file%.graph.json}"
  config_file="${base_name}.config.json"
  out_file="${base_name}.json"
  if test -f "$out_file"
  then
    echo "$out_file exists."
    exit 1
  fi
  echo Merging $graph_file and $config_file into $out_file
  echo '{' > $out_file
  echo '  "modules": [' >> $out_file
  echo '    "map",' >> $out_file
  echo '    "navigation"' >> $out_file
  echo '  ],' >> $out_file
  echo '  "graph": {' >> $out_file
  cat $graph_file | sed 's/^/  /' | tail -n +2 | head -n -1 >> $out_file
  echo '  },' >> $out_file
  echo '  "config": {' >> $out_file
  cat $config_file | sed 's/^/  /' | tail -n +2 | head -n -1 >> $out_file
  echo '  }' >> $out_file
  echo '}' >> $out_file
done

echo ====
echo Done
