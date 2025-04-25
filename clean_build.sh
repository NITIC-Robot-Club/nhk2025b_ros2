#!/bin/bash

cd "$(dirname "$0")"
rm -rf build/ install/ log/
if ! colcon build --symlink-install; then
    echo "Build failed..."
    exit 1
fi
source install/setup.bash
echo "Build succeeded. Source the workspace with:"
echo "source $(pwd)/install/setup.bash"