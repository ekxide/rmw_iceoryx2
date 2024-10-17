// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_INTROSPECTION_MESSAGE_HPP_
#define RMW_IOX2_INTROSPECTION_MESSAGE_HPP_

#include "rmw/dynamic_message_type_support.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"

namespace rmw::iox2
{

bool is_message_type(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_array_type(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_string_type(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_trivially_copyable(const rosidl_typesupport_introspection_c__MessageMembers* members);

bool is_message_type(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_array_type(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_string_type(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_trivially_copyable(const rosidl_typesupport_introspection_cpp::MessageMembers* members);

bool is_trivially_copyable(const rosidl_message_type_support_t* type_support);
size_t message_size(const rosidl_message_type_support_t* type_support);

} // namespace rmw::iox2

#endif // RMW_IOX2_INTROSPECTION_MESSAGE_HPP_
