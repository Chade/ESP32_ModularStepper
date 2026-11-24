#!/usr/bin/env bash
set -e

# Directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$SCRIPT_DIR"

# Include paths for host compilation
FIXED_POINTS_INC="/home/stefan/Documents/Arduino/libraries/FixedPoints/src"
STUBS_INC="$SCRIPT_DIR/sandbox_stubs"

# Output executable name
TARGET="sandbox_test"

echo "[BUILD] Compiling pure C++ sandbox executable ($TARGET)..."
g++ -std=c++17 -pthread -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-format \
    -DIRAM_ATTR= \
    -DLOG_LOCAL_LEVEL=0 \
    -I"$STUBS_INC" \
    -I"$FIXED_POINTS_INC" \
    -I"$PROJECT_DIR" \
    -I"$PROJECT_DIR/src" \
    -I"$SCRIPT_DIR" \
    "$PROJECT_DIR/src/Generic/Driver.cpp" \
    "$PROJECT_DIR/src/Generic/Generator.cpp" \
    "$PROJECT_DIR/src/Generic/Motor.cpp" \
    "$PROJECT_DIR/src/Generic/Axis.cpp" \
    "$PROJECT_DIR/src/Generic/Axis_Rotary.cpp" \
    "$PROJECT_DIR/src/Generic/Axis_Linear.cpp" \
    "$PROJECT_DIR/src/Generic/Homing.cpp" \
    "$PROJECT_DIR/src/Generic/StopController.cpp" \
    sandbox_test.cpp -o "$TARGET"

echo "[EXEC] Running $TARGET..."
echo "--------------------------------------------------"
./"$TARGET"
