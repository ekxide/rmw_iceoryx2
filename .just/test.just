# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# =============================================================================
# Test Recipes
# =============================================================================

# Run the rmw_iceoryx2 tests.
[group('test')]
test-rmw *colcon_args:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    source install/setup.bash

    # Print the results even when a test fails, then report the failure.
    status=0
    colcon test --packages-select {{rmw}} --event-handlers console_direct+ --return-code-on-test-failure {{colcon_args}} || status=$?
    colcon test-result --verbose --test-result-base "build/{{rmw}}" || status=$?
    exit "$status"

# Run the rmw conformance tests against rmw_iceoryx2.
[group('test')]
test-conformance *colcon_args:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    export RMW_IMPLEMENTATION={{rmw}}
    source install/setup.bash

    # The suite is generated once per installed rmw implementation; only run the one for rmw_iceoryx2.
    # Print the results even when a test fails, then report the failure.
    status=0
    colcon test --packages-ignore {{other_rmw_packages}} --packages-select test_rmw_implementation --event-handlers console_direct+ --return-code-on-test-failure --ctest-args -R {{rmw}} {{colcon_args}} || status=$?
    colcon test-result --verbose --test-result-base build/test_rmw_implementation || status=$?
    exit "$status"
