// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

const char* const rmw_iox2_serialization_format = "iceoryx2";

extern "C" {

const char* rmw_get_serialization_format(void) {
    return rmw_iox2_serialization_format;
}

rmw_ret_t
rmw_serialization_support_init(const char* serialization_lib_name,
                               rcutils_allocator_t* allocator,
                               rosidl_dynamic_typesupport_serialization_support_t* serialization_support) // OUT
{
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_serialize(const void* ros_message,
                        const rosidl_message_type_support_t* type_support,
                        rmw_serialized_message_t* serialized_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------

    // Handle for fastrtps typesupport
    const rosidl_message_type_support_t* handle =
        get_message_typesupport_handle(type_support, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
    if (!handle) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get typesupport handle");
        return RMW_RET_ERROR;
    }

    // FastRTPS-specific callbacks
    const message_type_support_callbacks_t* callbacks =
        static_cast<const message_type_support_callbacks_t*>(handle->data);
    if (!callbacks) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get typesupport callbacks");
        return RMW_RET_ERROR;
    }

    // Prepare the buffer
    auto fast_buffer = eprosima::fastcdr::FastBuffer(reinterpret_cast<char*>(serialized_message->buffer),
                                                     serialized_message->buffer_capacity);
    auto serializer = eprosima::fastcdr::Cdr(
        fast_buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR);

    // Serialize ros message into target buffer
    try {
        callbacks->cdr_serialize(ros_message, serializer);
    }
    catch (std::exception& e) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to serialize");
        return RMW_RET_ERROR;
    }

    serialized_message->buffer_length = serializer.get_serialized_data_length();

    return RMW_RET_OK;
}

rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                          const rosidl_message_type_support_t* type_support,
                          void* ros_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------

    // Handle for fastrtps typesupport
    const rosidl_message_type_support_t* handle =
        get_message_typesupport_handle(type_support, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
    if (!handle) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get typesupport handle");
        return RMW_RET_ERROR;
    }

    // FastRTPS-specific callbacks
    const message_type_support_callbacks_t* callbacks =
        static_cast<const message_type_support_callbacks_t*>(handle->data);
    if (!callbacks) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get typesupport callbacks");
        return RMW_RET_ERROR;
    }

    // Prepare the buffer
    eprosima::fastcdr::FastBuffer buffer(const_cast<char*>(reinterpret_cast<const char*>(serialized_message->buffer)),
                                         serialized_message->buffer_capacity);
    eprosima::fastcdr::Cdr deserializer(
        buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR);

    // Deserialize ros message into target buffer
    try {
        callbacks->cdr_deserialize(deserializer, ros_message);
    }
    catch (std::exception& e) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to deserialize");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}
}
