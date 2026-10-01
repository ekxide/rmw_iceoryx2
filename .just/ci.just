# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# =============================================================================
# CI Recipes
# =============================================================================

# SPDX identifier every source file must carry (see check-license-headers).
license := "Apache-2.0 OR MIT"

# Check that every source file has the SPDX license header.
[group('ci')]
[no-exit-message]
check-license-headers:
    #!/usr/bin/env bash
    set -euo pipefail

    cd "{{justfile_directory()}}"

    # File patterns per comment style. TOML is left out on purpose.
    slash_comment=('*.c' '*.cpp' '*.h' '*.hpp' '*.inl' '*.h.in' '*.hpp.in' '*.rs')
    hash_comment=('*.sh' '*.py' '*.cmake' '*.cmake.in' '*CMakeLists.txt' '*.just')

    missing=0

    # check <comment prefix> <file patterns...>
    check() {
        local expected="$1 SPDX-License-Identifier: {{license}}"
        shift

        # Tracked files and new files that are not ignored; skips build output.
        while IFS= read -r file; do
            [[ -f "$file" ]] || continue # tracked, but deleted in the working tree
            if ! head -n 12 "$file" | grep -qxF "$expected"; then
                echo "missing license header: $file"
                missing=1
            fi
        done < <(git ls-files --cached --others --exclude-standard -- "$@")
    }

    check '//' "${slash_comment[@]}"
    check '#' "${hash_comment[@]}"

    if [[ "$missing" == 0 ]]; then
        printf '\033[1;32mAll checked files have a valid license header\033[0m\n'
    else
        printf '\033[1;33mThe listed files do not have a valid license header\033[0m\n'
    fi
    exit "$missing"

# Check that the commits which are not yet on `base` start with the issue number, e.g. "[#123] Short description".
[group('ci')]
[no-exit-message]
check-commit-messages base="origin/rolling":
    #!/usr/bin/env bash
    set -euo pipefail

    cd "{{justfile_directory()}}"

    invalid=()

    # Merge commits are skipped; their messages are generated.
    while IFS=$'\t' read -r hash subject; do
        if [[ ! "$subject" =~ ^\[#[0-9]+\]\ .+ ]]; then
            invalid+=("$hash  $subject")
        fi
    done < <(git log --no-merges --format='%h%x09%s' "{{base}}..HEAD")

    if [[ ${#invalid[@]} == 0 ]]; then
        printf '\033[1;32mAll checked commit messages have a valid format\033[0m\n'
        exit 0
    fi

    printf '\033[1;33mCommit messages with an invalid format:\033[0m\n\n'
    printf '    %s\n' "${invalid[@]}"
    printf '\nExpected format: [#<issue-number>] Short description\n'
    exit 1

# Check that the C and C++ sources are formatted with clang-format.
[group('ci')]
[no-exit-message]
check-clang-format:
    #!/usr/bin/env bash
    set -euo pipefail

    cd "{{justfile_directory()}}"

    git ls-files | grep -E '\.(c|cc|cpp|cxx|inl|h|hh|hpp|hxx)$' | xargs clang-format -style=file --Werror --dry-run
    printf '\033[1;32mAll checked files are formatted\033[0m\n'
