# Build and run rmw_iceoryx2. Invoke from the workspace root:
#   just -f src/rmw_iceoryx2/justfile <recipe>

ws := invocation_directory()
rmw := "rmw_iceoryx2_cxx"
examples := justfile_directory() / "examples"
benchmark := justfile_directory() / "benchmark"

# rmw_iceoryx2 + ROS 2 CLI packages
minimal_packages := "ros2cli_common_extensions rmw_iceoryx2_cxx"

_default:
    @just --justfile {{justfile()}} --list

# Disable colcon-cargo's Cargo-workspace discovery. It enumerates every member of
# the vendored iceoryx2 cargo workspace as a colcon package and folds their
# dev-dependencies into the build order, creating `*-tests-common` cycles that make
# `colcon build` fail to order packages topologically. 
colcon_blocklist := "colcon_core.package_discovery.cargo_workspace:colcon_core.package_identification.cargo_workspace"

# Build rmw_iceoryx2 and the ROS 2 CLI.
build *extra_packages:
    #!/usr/bin/env bash
    set -euo pipefail

    cd "{{ws}}"
    export COLCON_EXTENSION_BLOCKLIST="{{colcon_blocklist}}"
    RMW_IMPLEMENTATION={{rmw}} colcon build --symlink-install --packages-up-to {{minimal_packages}} {{extra_packages}}

# Build the packages required to run an example.
build-example example:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    example_justfile="{{examples}}/{{example}}/justfile"
    [[ -f "$example_justfile" ]] || { echo "unknown example '{{example}}'" >&2; exit 1; }

    just -f "$example_justfile" build
    mkdir -p build && touch "build/.{{example}}.built"

# Run an example with the given configuration (see list-examples).
run-example example config="":
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    tmux_dir="{{examples}}/{{example}}/tmux"
    [[ -d "$tmux_dir" ]] || { echo "unknown example '{{example}}'" >&2; exit 1; }

    if [[ ! -f "build/.{{example}}.built" ]]; then
        echo "example '{{example}}' not built - run: just -f {{justfile()}} build-example {{example}}" >&2
        exit 1
    fi
    if ! command -v tmux >/dev/null; then
        echo "tmux is required to run examples" >&2
        exit 1
    fi

    mapfile -t configs < <(find "$tmux_dir" -maxdepth 1 -name '*.tmux' -printf '%f\n' | sed 's/\.tmux$//' | sort)
    [[ ${#configs[@]} -gt 0 ]] || { echo "no configurations defined for '{{example}}'" >&2; exit 1; }
    config="{{config}}"
    if [[ -z "$config" ]]; then
        if [[ ${#configs[@]} -eq 1 ]]; then
            config="${configs[0]}"
        else
            echo "example '{{example}}' has multiple configurations:" >&2
            printf '  %s\n' "${configs[@]}" >&2
            echo "specify one: just -f {{justfile()}} run-example {{example}} <config>" >&2
            exit 1
        fi
    fi

    script="$tmux_dir/$config.tmux"
    [[ -f "$script" ]] || { echo "unknown configuration '$config' for '{{example}}'" >&2; exit 1; }

    WORKSPACE_ROOT="{{ws}}" exec bash "$script"

# List the runnable examples and their configurations.
# Build the packages required to run the benchmarks.
build-benchmark:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    just -f "{{benchmark}}/justfile" build
    mkdir -p build && touch "build/.benchmark.built"

# Run a benchmark pairing, e.g.: run-benchmark ros2-to-ros2 rate=100 count=1000
run-benchmark pairing *parameters:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    if [[ ! -f "build/.benchmark.built" ]]; then
        echo "benchmarks not built - run: just -f {{justfile()}} build-benchmark" >&2
        exit 1
    fi

    just -f "{{benchmark}}/justfile" "{{pairing}}" {{parameters}}

list-benchmarks:
    #!/usr/bin/env bash
    set -eo pipefail

    printf '\033[2mpairing · named parameters: rate (Hz), count (samples), warmup (excluded from stats)\033[0m\n\n'
    just -f "{{benchmark}}/justfile" --summary | tr ' ' '\n' | grep -v '^build$\|^_' | sed 's/^/    /'
    printf '\nexample: just -f %s run-benchmark ros2-to-ros2 rate=100 count=1000\n' "{{justfile()}}"

list-examples:
    #!/usr/bin/env bash
    set -eo pipefail
    shopt -s nullglob

    printf '\033[2mexample\n    config\033[0m\n\n'
    for tmux_dir in "{{examples}}"/*/tmux; do
        printf '\033[1m%s\033[0m\n' "$(basename "$(dirname "$tmux_dir")")"
        for config in "$tmux_dir"/*.tmux; do
            echo "    $(basename "${config%.tmux}")"
        done
    done
