# Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
# 
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: Apache-2.0 OR MIT

#!/bin/bash

set -e

COLOR_RESET='\033[0m'
COLOR_GREEN='\033[1;32m'
COLOR_YELLOW='\033[1;33m'

cd $(git rev-parse --show-toplevel)

RET_VAL=0

check_license_header() {
    FILES=$(find . -type f -iwholename "${FILE_SUFFIX}" -not -path "./target/*" )

    for FILE in $FILES
    do
        HAS_CORRECT_HEADER=$(head -n 12 $FILE | grep -E "^$COMMENT_SYMBOL_GREP SPDX-License-Identifier: Apache-2.0 OR MIT\$" | wc -l)

        if [[ "$HAS_CORRECT_HEADER" != "1" ]]
        then
            echo "The file '$FILE' has a wrong license header."
            RET_VAL=1
        fi
    done
}

check_shell() {
    FILE_SUFFIX="*.sh"
    COMMENT_SYMBOL="#"
    COMMENT_SYMBOL_GREP="#"
    check_license_header
}

check_toml() {
    FILE_SUFFIX="*.toml"
    COMMENT_SYMBOL="#"
    COMMENT_SYMBOL_GREP="#"
    check_license_header
}

check_c_cpp() {
    FILE_SUFFIX="*.h"
    COMMENT_SYMBOL="\/\/"
    COMMENT_SYMBOL_GREP="//"
    check_license_header
    FILE_SUFFIX="*.h.in"
    check_license_header
    FILE_SUFFIX="*.c"
    check_license_header
    FILE_SUFFIX="*.hpp"
    check_license_header
    FILE_SUFFIX="*.hpp.in"
    check_license_header
    FILE_SUFFIX="*.inl"
    check_license_header
    FILE_SUFFIX="*.cpp"
    check_license_header
}

check_cmake() {
    FILE_SUFFIX="*.cmake"
    COMMENT_SYMBOL="#"
    COMMENT_SYMBOL_GREP="#"
    check_license_header
    FILE_SUFFIX="*.cmake.in"
    check_license_header
    FILE_SUFFIX="*CMakeLists.txt"
    check_license_header
}

check_shell
check_c_cpp
check_cmake

# no toml check for now
# it is usually only some configuration files which can be used without copyright notice
# check_toml

if [[ "$RET_VAL" == "0" ]]
then
    echo -e "${COLOR_GREEN}All checked files have a valid license header${COLOR_RESET}"
else
    echo -e "${COLOR_YELLOW}The listed files don't have a valid license header${COLOR_RESET}"
fi

exit $RET_VAL
