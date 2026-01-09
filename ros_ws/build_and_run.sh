#!/bin/bash

set -e  # 任一步失败就退出

WS=~/scut_lessondesign/ros_ws
BUILD_DIR=$WS/build
EXEC=arm_ui_node

echo ">>> Enter workspace: $WS"
cd "$WS"

echo ">>> Build project"
cmake --build "$BUILD_DIR" -j

echo ">>> Enter build directory"
cd "$BUILD_DIR"

if [ ! -f "$EXEC" ]; then
    echo "ERROR: executable '$EXEC' not found in build directory"
    exit 1
fi

echo ">>> Run $EXEC"
./"$EXEC"

