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

# Disable colcon-cargo's Cargo-workspace discovery. It enumerates every member of
# the vendored iceoryx2 cargo workspace as a colcon package and folds their
# dev-dependencies into the build order, creating `*-tests-common` cycles that make
# `colcon build` fail to order packages topologically.
colcon_blocklist := "colcon_core.package_discovery.cargo_workspace:colcon_core.package_identification.cargo_workspace"
