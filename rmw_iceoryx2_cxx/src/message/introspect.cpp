// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/message/introspect.hpp"

#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

namespace rmw::iox2
{

bool is_message(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return member->type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE;
}

bool is_fixed_array(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return member->is_array_ && member->array_size_ > 0 && !member->is_upper_bound_;
}

bool is_dynamic_array(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return member->is_array_ && (member->array_size_ == 0 || member->is_upper_bound_);
}

bool is_dynamic_string(const rosidl_typesupport_introspection_c__MessageMember* member) {
    return member->type_id_ == rosidl_typesupport_introspection_c__ROS_TYPE_STRING;
}

bool is_pod(const rosidl_typesupport_introspection_c__MessageMembers* members) {
    if (members == nullptr) {
        return false;
    }

    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto* member = members->members_ + i;

        if (is_dynamic_array(member) || is_dynamic_string(member)) {
            return false;
        }
        if (is_message(member)) {
            if (!member->members_ || !member->members_->data) {
                return false;
            }
            if (!is_pod(
                    static_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(member->members_->data))) {
                return false;
            }
        }
    }
    return true;
}

bool is_message(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return member->type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
}

bool is_fixed_array(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return member->is_array_ && member->array_size_ > 0 && !member->is_upper_bound_;
}

bool is_dynamic_array(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return member->is_array_ && (member->array_size_ == 0 || member->is_upper_bound_);
}

bool is_dynamic_string(const rosidl_typesupport_introspection_cpp::MessageMember* member) {
    return member->type_id_ == ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
}

bool is_pod(const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
    if (members == nullptr)
        return false;

    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto* member = members->members_ + i;

        if (is_dynamic_array(member) || is_dynamic_string(member)) {
            return false;
        }
        if (is_message(member)) {
            if (!member->members_ || !member->members_->data)
                return false;
            if (!is_pod(
                    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member->members_->data))) {
                return false;
            };
        }
    }
    return true;
}

bool is_pod(const rosidl_message_type_support_t* type_support) {
    if (auto handle = get_message_typesupport_handle(type_support,
                                                     rosidl_typesupport_introspection_cpp::typesupport_identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(handle->data);
        return is_pod(members);
    }
    return false;
}

size_t message_size(const rosidl_message_type_support_t* type_support) {
    // Try C++ typesupport first
    if (auto handle = get_message_typesupport_handle(type_support,
                                                     rosidl_typesupport_introspection_cpp::typesupport_identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(handle->data);
        return members->size_of_;
    }

    // Try C typesupport if C++ failed
    if (auto handle = get_message_typesupport_handle(type_support, rosidl_typesupport_introspection_c__identifier)) {
        auto members = static_cast<const rosidl_typesupport_introspection_c__MessageMembers*>(handle->data);
        return members->size_of_;
    }

    RMW_IOX2_CHAIN_ERROR_MSG("failed to determine message size");
    return 0;
}

size_t serialized_message_size(const void* ros_message, const rosidl_message_type_support_t* type_support) {
    if (!type_support || !type_support->data) {
        return 0;
    }
    if (auto handle =
            get_message_typesupport_handle(type_support, rosidl_typesupport_fastrtps_cpp::typesupport_identifier)) {
        auto callbacks = static_cast<const message_type_support_callbacks_t*>(handle->data);
        return 4 + callbacks->get_serialized_size(ros_message); // 4 bytes for CDR header
    }
    return 0;
}

} // namespace rmw::iox2
