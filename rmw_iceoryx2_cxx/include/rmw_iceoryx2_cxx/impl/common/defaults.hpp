// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rcutils/types/string_array.h"

#ifndef RMW_IOX2_COMMON_DEFAULTS_HPP_
#define RMW_IOX2_COMMON_DEFAULTS_HPP_

constexpr rcutils_string_array_t RCUTILS_STRING_ARRAY_ZERO = {
    0,                                            // size
    nullptr,                                      // data
    {nullptr, nullptr, nullptr, nullptr, nullptr} // allocator
};

#endif
