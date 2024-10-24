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

static const size_t MAX_ERROR_MSG_LENGTH = 4096;

} // namespace rmw::iox2

extern "C" {

#define RMW_IOX2_CHECK_ALLOCATOR(allocator, fail_statement) RCUTILS_CHECK_ALLOCATOR(allocator, fail_statement)

#define RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH(element, type_id, expected_type_id, on_failure)                          \
    RMW_CHECK_TYPE_IDENTIFIERS_MATCH(element, type_id, expected_type_id, on_failure)

#define RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(argument, error_return_value)                                                 \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(argument, error_return_value)


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

#define RMW_IOX2_OK_OR_RETURN(result)                                                                                  \
    if (result != RMW_RET_OK) {                                                                                        \
        return result;                                                                                                 \
    }
}
#endif
