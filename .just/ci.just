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
