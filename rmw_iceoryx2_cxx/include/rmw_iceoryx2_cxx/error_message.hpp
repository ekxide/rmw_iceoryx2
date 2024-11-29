// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_ERROR_HANDLING_HPP_
#define RMW_IOX2_ERROR_HANDLING_HPP_

#include "rmw/allocators.h"
#include "rmw/error_handling.h"

#include <cstddef>

namespace rmw::iox2
{

static const size_t MAX_ERROR_MSG_LENGTH = 4096;

} // namespace rmw::iox2


#define RMW_IOX2_CHAIN_ERROR_MSG(msg)                                                                                  \
    do {                                                                                                               \
        if (rcutils_error_is_set()) {                                                                                  \
            rcutils_error_string_t error_string = rcutils_get_error_string();                                          \
            rcutils_reset_error();                                                                                     \
            RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING("%s: %s", msg, error_string.str);                                 \
        } else {                                                                                                       \
            RCUTILS_SET_ERROR_MSG(msg);                                                                                \
        }                                                                                                              \
    } while (0)

#define RMW_IOX2_CHAIN_ERROR_MSG_WITH_FORMAT_STRING(format, ...)                                                       \
    do {                                                                                                               \
        if (rcutils_error_is_set()) {                                                                                  \
            rcutils_error_string_t error_string = rcutils_get_error_string();                                          \
            rcutils_reset_error();                                                                                     \
            char formatted_msg[rmw::iox2::MAX_ERROR_MSG_LENGTH];                                                       \
            snprintf(formatted_msg, sizeof(formatted_msg), format, __VA_ARGS__);                                       \
            RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING("%s: %s", formatted_msg, error_string.str);                       \
        } else {                                                                                                       \
            RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(format, __VA_ARGS__);                                             \
        }                                                                                                              \
    } while (0)
#endif
