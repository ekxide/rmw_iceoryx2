// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/dynamic_message_type_support.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/introspection/message.hpp"

const char* const rmw_iox2_serialization_format = "iceoryx2";

extern "C" {

const char* rmw_get_serialization_format(void) {
    return rmw_iox2_serialization_format;
}

rmw_ret_t rmw_serialize(const void* ros_message,
                        const rosidl_message_type_support_t* type_support,
                        rmw_serialized_message_t* serialized_message) {
    using rmw::iox2::is_trivially_copyable;
    using rmw::iox2::message_size;

    RCUTILS_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

    if (!is_trivially_copyable(type_support)) {
        RMW_IOX2_CHAIN_ERROR_MSG("non-trivially-copiable types are unsupported");
        return RMW_RET_UNSUPPORTED;
    }

    auto size = message_size(type_support);
    if (auto result = rmw_serialized_message_resize(serialized_message, size) != RMW_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to resize serialized message");
        return result;
    }

    serialized_message->buffer_length = size;
    memcpy(serialized_message->buffer, ros_message, size);

    return RMW_RET_OK;
}

rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                          const rosidl_message_type_support_t* type_support,
                          void* ros_message) {
    using rmw::iox2::is_trivially_copyable;

    RCUTILS_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

    if (!is_trivially_copyable(type_support)) {
        RMW_IOX2_CHAIN_ERROR_MSG("non-trivially-copiable types are unsupported");
        return RMW_RET_UNSUPPORTED;
    }

    memcpy(ros_message, serialized_message->buffer, serialized_message->buffer_length);

    return RMW_RET_OK;
}

rmw_ret_t
rmw_serialization_support_init(const char* serialization_lib_name,
                               rcutils_allocator_t* allocator,
                               rosidl_dynamic_typesupport_serialization_support_t* serialization_support) // OUT
{
    return RMW_RET_UNSUPPORTED;
}
}
