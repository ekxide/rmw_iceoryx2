# Build and run rmw_iceoryx2. Invoke from the workspace root:
#   just -f src/rmw_iceoryx2/justfile <recipe>

ws := invocation_directory()
rmw := "rmw_iceoryx2_cxx"

# rmw_iceoryx2 + ROS 2 CLI packages
packages := "ros2cli_common_extensions rmw_iceoryx2_cxx"

_default:
    @just --justfile {{justfile()}} --list

# Build rmw_iceoryx2 and the ROS 2 CLI.
build *args:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ws}}"
    RMW_IMPLEMENTATION={{rmw}} colcon build --symlink-install --packages-up-to {{packages}} {{args}}
