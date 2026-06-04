#!/bin/bash

# Exit on any error
set -e

# Default PX4 directory
DEFAULT_PX4_DIR="$HOME/PX4-Autopilot"

PX4_DIR="$DEFAULT_PX4_DIR"
SHOW_HELP=false

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -p|--px4-dir)
            PX4_DIR="$2"
            shift
            shift
            ;;
        -h|--help)
            SHOW_HELP=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            SHOW_HELP=true
            shift
            ;;
    esac
done

if [ "$SHOW_HELP" = true ]; then
    echo "Usage: $0 [OPTIONS]"
    echo "Copy PX4 model files from PX4-Autopilot into this ROS 2 package."
    echo ""
    echo "Options:"
    echo "  -p, --px4-dir PATH    Path to PX4-Autopilot directory"
    echo "                         Default: $DEFAULT_PX4_DIR"
    echo "  -h, --help            Show this help message"
    exit 0
fi

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot directory not found at: $PX4_DIR"
    echo "Please specify the correct path using -p or --px4-dir"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"

AIRFRAME_SRC="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_mono"
AIRFRAME_DST="$PACKAGE_DIR/models/px4_models/airframes/4022_gz_x500_depth_mono"

GZ_MODEL_SRC="$PX4_DIR/Tools/simulation/gz/models/x500_depth_mono"
GZ_MODEL_DST="$PACKAGE_DIR/models/px4_models/gz_models"

echo "Using PX4-Autopilot directory: $PX4_DIR"
echo "Using package directory: $PACKAGE_DIR"

mkdir -p "$PACKAGE_DIR/models/px4_models/airframes"
mkdir -p "$PACKAGE_DIR/models/px4_models/gz_models"

echo "Copying PX4 model files into package..."

if [ -f "$AIRFRAME_SRC" ]; then
    cp "$AIRFRAME_SRC" "$AIRFRAME_DST"
    echo "Copied airframe: 4022_gz_x500_depth_mono"
else
    echo "Error: Expected airframe not found:"
    echo "$AIRFRAME_SRC"
    echo ""
    echo "Make sure the custom TerraDrop-PX4 airframe exists in PX4-Autopilot."
    exit 1
fi

if [ -f "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" ]; then
    cp "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt" \
       "$PACKAGE_DIR/models/px4_models/airframes/"
    echo "Copied airframes CMakeLists.txt"
fi

if [ -d "$GZ_MODEL_SRC" ]; then
    rm -rf "$GZ_MODEL_DST/x500_depth_mono"
    cp -r "$GZ_MODEL_SRC" "$GZ_MODEL_DST/"
    echo "Copied Gazebo model: x500_depth_mono"
else
    echo "Warning: Gazebo model not found:"
    echo "$GZ_MODEL_SRC"
fi

echo "Setup complete. PX4 model files have been copied into the package."