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

    RMW_IOX2_CHAIN_ERROR_MSG("failed to determine message size");
    return 0;
}

constexpr size_t introspect_member_size(const rosidl_typesupport_introspection_cpp::MessageMember& member) {
    size_t size = 0;

    switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        size = 1;
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
        size = 2;
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        size = 4;
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        size = 8;
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        size = 16;
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        // TODO: Magic number
        size = 1024;
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
        const rosidl_message_type_support_t* nested_ts =
            static_cast<const rosidl_message_type_support_t*>(member.members_->data);
        size = introspect_message_size(nested_ts);
        break;
    }
    default:
        size = 0;
        break;
    }

    if (member.array_size_ > 0 || member.is_array_) {
        // TODO: Magic number
        size_t array_size = member.array_size_ > 0 ? member.array_size_ : 1024;
        size *= array_size;
    }

    return size;
}

constexpr size_t introspect_message_size(const rosidl_message_type_support_t* ts) {
    const auto* members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(ts->data);

    size_t total_size = 0;

    for (size_t i = 0; i < members->member_count_; ++i) {
        const auto& member = members->members_[i];
        total_size += introspect_member_size(member);
    }

    return total_size;
}

} // namespace rmw::iox2
