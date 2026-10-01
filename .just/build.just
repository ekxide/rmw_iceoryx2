# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# =============================================================================
# Build Recipes
# =============================================================================

# Build rmw_iceoryx2 and the ROS 2 CLI.
[group('build')]
build *extra_packages:
    #!/usr/bin/env bash
    set -euo pipefail

    cd "{{ws}}"
    export COLCON_EXTENSION_BLOCKLIST="{{colcon_blocklist}}"
    RMW_IMPLEMENTATION={{rmw}} colcon build --symlink-install --packages-ignore {{other_rmw_packages}} --packages-up-to {{rmw}} ros2cli_common_extensions {{extra_packages}}

# Build only rmw_iceoryx2 and its tests; extra arguments are passed to colcon.
[group('build')]
build-rmw *colcon_args:
    #!/usr/bin/env bash
    set -euo pipefail

    cd "{{ws}}"
    export COLCON_EXTENSION_BLOCKLIST="{{colcon_blocklist}}"
    colcon build --symlink-install --packages-up-to {{rmw}} {{colcon_args}}
