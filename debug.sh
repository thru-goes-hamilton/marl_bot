#!/bin/bash

# List of keywords
keywords=(
    "ament_cmake"
    "ament_cmake_python"
    "ament_cmake_libraries"
    "ament_cmake_core"
    "ament_cmake_export_dependencies"
    "ament_cmake_export_definitions"
    "ament_cmake_export_include_directories"
    "ament_cmake_export_interfaces"
    "ament_cmake_export_libraries"
    "ament_cmake_export_link_flags"
    "ament_cmake_export_targets"
    "ament_cmake_gen_version_h"
    "ament_cmake_target_dependencies"
    "ament_cmake_include_directories"
    "ament_cmake_test"
    "ament_cmake_version"
    "ament_lint_auto"
)

# Loop through each keyword
for keyword in "${keywords[@]}"; do
    echo "Processing: $keyword"
    
    # Find the Config.cmake file
    config_path=$(find /opt/ros/humble -name "${keyword}Config.cmake" 2>/dev/null)

    if [ -z "$config_path" ]; then
        echo "❌ Config file not found for $keyword, skipping..."
        continue
    fi

    # Extract the directory path
    dir_path=$(dirname "$config_path")

    # Export the variable
    export_var="${keyword}_DIR"
    export "$export_var=$dir_path"

    # Run colcon build
    echo "✅ Building with: -D${export_var}=${dir_path}"
    sudo colcon build --cmake-args "-D${export_var}=${dir_path}"
done

echo "✅ All processes completed!"
