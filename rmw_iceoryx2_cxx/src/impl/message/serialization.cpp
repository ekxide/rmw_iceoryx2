// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/message/serialization.hpp"
#include "iox/expected.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>

namespace rmw::iox2
{

/// @brief Serializes a ROS message into a buffer using FastDDS CDR serialization
/// @param[in] ros_message Pointer to the ROS message to serialize
/// @param[in] type_support Type support information for the message
/// @param[out] buffer Buffer to store the serialized message
/// @param[in] buffer_size Size of the provided buffer
/// @return Success if serialization was successful, error code otherwise
auto serialize(const void* ros_message,
               const rosidl_message_type_support_t* type_support,
               void* buffer,
               size_t buffer_size) -> iox::expected<void, SerializationError> {
    const message_type_support_callbacks_t* callbacks =
        static_cast<const message_type_support_callbacks_t*>(type_support->data);
    if (!callbacks) {
        return iox::err(SerializationError::TYPESUPPORT_FAILURE);
    }

    auto fast_buffer = eprosima::fastcdr::FastBuffer(reinterpret_cast<char*>(buffer), buffer_size);
    auto serializer = eprosima::fastcdr::Cdr(
        fast_buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR);

    try {
        callbacks->cdr_serialize(ros_message, serializer);
    }
    catch (std::exception& e) {
        return iox::err(SerializationError::SERIALIZATION_FAILURE);
    }

    return iox::ok();
}

/// @brief Deserializes a buffer containing a serialized ROS message using FastDDS CDR deserialization
/// @param[in] serialized_message Pointer to the buffer containing the serialized message
/// @param[in] serialized_size Size of the serialized message
/// @param[in] type_support Type support information for the message
/// @param[out] ros_message Pointer to store the target ros message to deserialize into
/// @return Success if deserialization was successful, error code otherwise
auto deserialize(const void* serialized_message,
                 const size_t serialized_size,
                 const rosidl_message_type_support_t* type_support,
                 void* ros_message) -> iox::expected<void, DeserializationError> {
    const message_type_support_callbacks_t* callbacks =
        static_cast<const message_type_support_callbacks_t*>(type_support->data);
    if (!callbacks) {
        return iox::err(DeserializationError::TYPESUPPORT_FAILURE);
    }

    eprosima::fastcdr::FastBuffer buffer(const_cast<char*>(reinterpret_cast<const char*>(serialized_message)),
                                         serialized_size);
    eprosima::fastcdr::Cdr deserializer(
        buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::DDS_CDR);

    try {
        callbacks->cdr_deserialize(deserializer, ros_message);
    }
    catch (std::exception& e) {
        return iox::err(DeserializationError::DESERIALIZATION_FAILURE);
    }

    return iox::ok();
}

} // namespace rmw::iox2
