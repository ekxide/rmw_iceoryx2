// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/dynamic_message_type_support.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"

const char* const rmw_iox2_serialization_format = "iceoryx2";

extern "C" {

const char* rmw_get_serialization_format(void) {
    return rmw_iox2_serialization_format;
}

rmw_ret_t rmw_serialize(const void* ros_message,
                        const rosidl_message_type_support_t* type_support,
                        rmw_serialized_message_t* serialized_message) {
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_ERROR);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_ERROR);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_ERROR);

    IOX_TODO();
}

rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                          const rosidl_message_type_support_t* type_support,
                          void* ros_message) {
    IOX_TODO();
}

rmw_ret_t
rmw_serialization_support_init(const char* serialization_lib_name,
                               rcutils_allocator_t* allocator,
                               rosidl_dynamic_typesupport_serialization_support_t* serialization_support) // OUT
{
    IOX_TODO();
}
}
