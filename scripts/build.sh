#!/bin/bash
# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Delete the build directory if it exists for a clean build
echo "Removing existing build directory for clean build..."
rm -rf "$PROJECT_DIR/build"

# Create and navigate to the build directory
echo "Creating fresh build directory..."
mkdir -p "$PROJECT_DIR/build"
cd "$PROJECT_DIR/build"

# Run cmake and make
cmake ..
make -j$(nproc)
