// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/introspection/message.hpp"

#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

namespace rmw::iox2
{

bool is_message_type(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return member->type_id_ == ::rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE;
}

bool is_array_type(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return (member->is_array_ && !(member->array_size_ > 0 && !member->is_upper_bound_));
}

bool is_string_type(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return member->type_id_ == ::rosidl_typesupport_introspection_c__ROS_TYPE_STRING;
}

bool is_trivially_copyable(const rosidl_typesupport_introspection_c__MessageMembers* members) {
    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto* member = members->members_ + i;
        if (is_message_type(member)) {
            return is_trivially_copyable(
                static_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member->members_->data));
        } else if (is_array_type(member) || is_string_type(member)) {
            return false;
        }
    }
    return true;
}

bool is_message_type(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return member->type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
}

bool is_array_type(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return (member->is_array_ && !(member->array_size_ > 0 && !member->is_upper_bound_));
}

bool is_string_type(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return member->type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
}

bool is_trivially_copyable(const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto* member = members->members_ + i;
        if (is_message_type(member)) {
            return is_trivially_copyable(
                static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member->members_->data));
        } else if (is_array_type(member) || is_string_type(member)) {
            return false;
        }
    }
    return true;
}

bool is_trivially_copyable(const rosidl_message_type_support_t* type_support) {
    if (auto handle = get_message_typesupport_handle(type_support,
                                                     rosidl_typesupport_introspection_cpp::typesupport_identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(handle->data);
        return is_trivially_copyable(members);
    }
    if (auto handle = get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_c__identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(handle->data);
        return is_trivially_copyable(members);
    }
    return false;
}

size_t message_size(const rosidl_message_type_support_t* type_support) {
    if (auto handle = get_message_typesupport_handle(type_support,
                                                     rosidl_typesupport_introspection_cpp::typesupport_identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(handle->data);
        return members->size_of_;
    }
    if (auto handle = get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_c__identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(handle->data);
        return members->size_of_;
    }

    RMW_IOX2_SET_ERROR_MSG("failed to determine message size");
    return 0;
}

} // namespace rmw::iox2
