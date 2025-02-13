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

#include "rmw/dynamic_message_type_support.h"
#include "rmw/visibility_control.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace rmw::iox2
{

bool is_message(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_fixed_array(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_dynamic_array(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_dynamic_string(const rosidl_typesupport_introspection_c__MessageMember* member);
bool is_self_contained(const rosidl_typesupport_introspection_c__MessageMembers* members);

bool is_message(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_fixed_array(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_dynamic_array(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_dynamic_string(const rosidl_typesupport_introspection_cpp::MessageMember* member);
bool is_self_contained(const rosidl_typesupport_introspection_cpp::MessageMembers* members);

bool is_self_contained(const rosidl_message_type_support_t* type_support);
RMW_PUBLIC size_t message_size(const rosidl_message_type_support_t* type_support);
RMW_PUBLIC size_t serialized_message_size(const void* ros_message, const rosidl_message_type_support_t* type_support);

} // namespace rmw::iox2

#endif // RMW_IOX2_INTROSPECTION_MESSAGE_HPP_
