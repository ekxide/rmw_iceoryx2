// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_MESSAGE_INTROSPECTION_HPP_
#define RMW_IOX2_MESSAGE_INTROSPECTION_HPP_

#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include <string>

namespace rmw::iox2
{

/// @brief True if the message has no dynamic (heap-allocated) content.
/// @details A self-contained message can be memcpy into an `iceoryx2`
///          payload. Other messages must be serialized.
RMW_PUBLIC auto is_self_contained(const rosidl_message_type_support_t* type_support) -> bool;

/// @brief The in-memory size of the message's C/C++ struct (`sizeof(T)`).
RMW_PUBLIC auto message_size(const rosidl_message_type_support_t* type_support) -> size_t;

/// @brief The rosidl provided type name `<package>/msg/<Type>` derived from introspection.
RMW_PUBLIC auto message_type_name(const rosidl_message_type_support_t* type_support) -> std::string;

/// @brief The REP-2011 type hash from the typesupport, or `nullopt` if it does
///        not provide one.
RMW_PUBLIC auto message_type_hash(const rosidl_message_type_support_t* type_support)
    -> ::iox2::bb::Optional<rosidl_type_hash_t>;

/// @brief The per-instance serialized size in bytes, including the 4-byte
///        CDR encapsulation header.
RMW_PUBLIC auto serialized_message_size(const void* ros_message, const rosidl_message_type_support_t* type_support)
    -> size_t;

} // namespace rmw::iox2

#endif // RMW_IOX2_MESSAGE_INTROSPECTION_HPP_
