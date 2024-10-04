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

#include "rmw/check_type_identifiers_match.h"
#include "rmw/error_handling.h"
#include <stddef.h>

namespace rmw::iox2
{

static const size_t MAX_ERROR_MSG_LENGTH = 1024;

} // namespace rmw::iox2

extern "C" {

#define RMW_IOX2_CHECK_ALLOCATOR(allocator, fail_statement) RCUTILS_CHECK_ALLOCATOR(allocator, fail_statement)

#define RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH(element, type_id, expected_type_id, on_failure)                          \
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH(element, type_id, expected_type_id, on_failure)

#define RMW_IOX2_CHECK_FOR_NULL(value, msg, on_error)                                                                  \
    do {                                                                                                               \
        if (nullptr == value) {                                                                                        \
            RMW_IOX2_CHAIN_ERROR_MSG(msg);                                                                             \
            on_error;                                                                                                  \
        }                                                                                                              \
    } while (0)

#define RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(argument, error_return_value)                                                 \
    RMW_IOX2_CHECK_FOR_NULL(argument, #argument " argument is null", return error_return_value)

#define RMW_IOX2_GET_ERROR_MSG()                                                                                       \
    [&]() -> std::string {                                                                                             \
        const rmw_error_state_t* error = rmw_get_error_state();                                                        \
        if (error) {                                                                                                   \
            char buffer[rmw::iox2::MAX_ERROR_MSG_LENGTH];                                                              \
            snprintf(buffer,                                                                                           \
                     sizeof(buffer),                                                                                   \
                     "Error:\n  %s (%s:%d)\n",                                                                         \
                     error->message,                                                                                   \
                     error->file,                                                                                      \
                     static_cast<int>(error->line_number));                                                            \
            rmw_reset_error();                                                                                         \
            return std::string(buffer);                                                                                \
        }                                                                                                              \
        return "";                                                                                                     \
    }()

#define RMW_IOX2_CHAIN_ERROR_MSG(msg)                                                                                  \
    do {                                                                                                               \
        char buf[rmw::iox2::MAX_ERROR_MSG_LENGTH];                                                                     \
        const rmw_error_state_t* previous_error = rmw_get_error_state();                                               \
        if (previous_error && strlen(previous_error->message) > 0) {                                                   \
            snprintf(buf, sizeof(buf), "%s: %s", msg, previous_error->message);                                        \
        } else {                                                                                                       \
            strncpy(buf, msg, sizeof(buf) - 1);                                                                        \
            buf[sizeof(buf) - 1] = '\0';                                                                               \
        }                                                                                                              \
        rmw_reset_error();                                                                                             \
        rmw_set_error_state(buf, __FILE__, __LINE__);                                                                  \
    } while (0)
}

#define RMW_IOX2_OK_OR_RETURN(result)                                                                                  \
    if (result != RMW_RET_OK) {                                                                                        \
        return result;                                                                                                 \
    }

#endif
