#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Usage: $0 <rosbag_directory>"
  exit 1
fi

BAG_DIR="$1"

ros2 bag play "$BAG_DIR" --remap $(
  ros2 bag info "$BAG_DIR" \
    | awk '
      /Topic: \/localization\//  {print $2":=/localization_dummy"substr($2,14)}
      /Topic: \/visualization\// {print $2":=/visualization_dummy"substr($2,16)}
    '
) /tf:=/tf_dummy /tf_static:=/tf_static_dummy
