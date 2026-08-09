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
g++ -std=c++17 -Wall -Wextra -Wno-unused-parameter -Wno-format \
    -DIRAM_ATTR= \
    -I"$STUBS_INC" \
    -I"$FIXED_POINTS_INC" \
    -I"$PROJECT_DIR" \
    -I"$SCRIPT_DIR" \
    "$PROJECT_DIR/StepperDriver_Base.cpp" \
    "$PROJECT_DIR/StepperGenerator.cpp" \
    sandbox_test.cpp -o "$TARGET"

echo "[EXEC] Running $TARGET..."
echo "--------------------------------------------------"
./"$TARGET"

