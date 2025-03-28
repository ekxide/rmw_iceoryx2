// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/message/typesupport.hpp"
#include "rmw/error_handling.h"

const rosidl_message_type_support_t* get_handle(const rosidl_message_type_support_t* type_support,
                                                const char* identifier) {
    const rosidl_message_type_support_t* next = type_support;

    // Keep trying until we run out of type supports to check
    while (next != nullptr) {
        // Try to get handle from current type support
        if (auto ts = get_message_typesupport_handle(next, identifier); ts) {
            return ts;
        }
        rcutils_reset_error();

        // Move to next type support if there is one
        if (next->func) {
            next = next->func(next, identifier);
        } else {
            break;
        }
    }
    return nullptr;
}
