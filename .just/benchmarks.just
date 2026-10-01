# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# =============================================================================
# Benchmark Recipes
# =============================================================================

# Build the packages required to run the benchmarks.
[group('benchmarks')]
build-benchmark:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    just -f "{{benchmark}}/justfile" build
    mkdir -p build && touch "build/.benchmark.built"

# Run a benchmark pairing, e.g.: run-benchmark ros2-to-ros2 rate=100 count=1000
[group('benchmarks')]
run-benchmark pairing *parameters:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    if [[ ! -f "build/.benchmark.built" ]]; then
        echo "benchmarks not built - run: just -f {{justfile()}} build-benchmark" >&2
        exit 1
    fi

    just -f "{{benchmark}}/justfile" "{{pairing}}" {{parameters}}

# List the benchmark pairings and their parameters.
[group('benchmarks')]
list-benchmarks:
    #!/usr/bin/env bash
    set -eo pipefail

    printf '\033[2mpairing · named parameters: rate (Hz), count (samples), warmup (excluded from stats)\033[0m\n\n'
    just -f "{{benchmark}}/justfile" --summary | tr ' ' '\n' | grep -v '^build$\|^_' | sed 's/^/    /'
    printf '\nexample: just -f %s run-benchmark ros2-to-ros2 rate=100 count=1000\n' "{{justfile()}}"
