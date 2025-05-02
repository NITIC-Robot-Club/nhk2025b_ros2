#!/bin/bash

cd "$(dirname "$0")"

if ! colcon build --symlink-install; then
    echo "Build failed..."
    exit 1
fi

echo "Build succeeded. Source the workspace:"
echo ""
echo ""
echo "source $(pwd)/install/setup.bash"
echo ""
echo ""