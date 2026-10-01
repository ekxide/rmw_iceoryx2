# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# =============================================================================
# Shared Variables
# =============================================================================

ws := invocation_directory()
rmw := "rmw_iceoryx2_cxx"
examples := justfile_directory() / "examples"
benchmark := justfile_directory() / "benchmark"

# rmw_iceoryx2 + ROS 2 CLI packages
minimal_packages := "ros2cli_common_extensions rmw_iceoryx2_cxx"

# Disable colcon-cargo's Cargo-workspace discovery: it counts dev-dependencies as
# build-order dependencies, which makes the iceoryx2 crates cyclic and `colcon build`
# fail.
# Remove once colcon-cargo leaves dev-dependencies out of the build order and no
# longer claims the iceoryx2 workspace root and its member crates.
colcon_blocklist := "colcon_core.package_discovery.cargo_workspace:colcon_core.package_identification.cargo_workspace"

# The other rmw implementations.
other_rmw_packages := "rmw_connextdds rmw_connextddsmicro rmw_cyclonedds_cpp rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp rmw_zenoh_cpp"
