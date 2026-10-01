# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# =============================================================================
# Example Recipes
# =============================================================================

# Build the packages required to run an example.
[group('examples')]
build-example example:
    #!/usr/bin/env bash
    set -eo pipefail

    cd "{{ws}}"
    example_justfile="{{examples}}/{{example}}/justfile"
    [[ -f "$example_justfile" ]] || { echo "unknown example '{{example}}'" >&2; exit 1; }

    just -f "$example_justfile" build
    mkdir -p build && touch "build/.{{example}}.built"

# Run an example with the given configuration (see list-examples).
[group('examples')]
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
[group('examples')]
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
