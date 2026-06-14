// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"

#include "iox2/legacy/variant.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/message/typesupport.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "rosidl_runtime_c/type_hash.h"

namespace rmw::iox2
{

// Helpers --------------------------------------------------------------

namespace
{

namespace introspection_cpp = rosidl_typesupport_introspection_cpp;

using CMember = rosidl_typesupport_introspection_c__MessageMember;
using CMembers = rosidl_typesupport_introspection_c__MessageMembers;
using CppMember = introspection_cpp::MessageMember;
using CppMembers = introspection_cpp::MessageMembers;
using IntrospectionView = ::iox2::legacy::variant<const CppMembers*, const CMembers*>;

template <typename Member>
auto is_string(const Member* m) -> bool {
    // Both `string` and `wstring` own dynamically-sized heap buffers; for the
    // purpose of self-containedness they are equivalent.
    return m->type_id_ == introspection_cpp::ROS_TYPE_STRING || m->type_id_ == introspection_cpp::ROS_TYPE_WSTRING;
}

template <typename Member>
auto is_message(const Member* m) -> bool {
    return m->type_id_ == introspection_cpp::ROS_TYPE_MESSAGE;
}

template <typename Member>
auto is_fixed_array(const Member* m) -> bool {
    return m->is_array_ && m->array_size_ > 0 && !m->is_upper_bound_;
}

template <typename Member>
auto is_dynamic_array(const Member* m) -> bool {
    return m->is_array_ && (m->array_size_ == 0 || m->is_upper_bound_);
}

// A field has a runtime-determined size if its content lives on the heap.
template <typename Member>
auto has_dynamic_size(const Member* m) -> bool {
    return is_string(m) || is_dynamic_array(m);
}

template <typename Members>
auto is_self_contained_impl(const Members* members) -> bool {
    if (!members) {
        return false;
    }
    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto* member = members->members_ + i;

        if (has_dynamic_size(member)) {
            return false;
        }

        if (is_message(member)) {
            if (!member->members_ || !member->members_->data) {
                return false;
            }
            const auto* nested = static_cast<const Members*>(member->members_->data);
            if (!is_self_contained_impl(nested)) {
                return false;
            }
        }
    }
    return true;
}

auto find_introspection(const rosidl_message_type_support_t* ts) -> IntrospectionView {
    IntrospectionView view;
    if (auto handle = get_message_typesupport_handle(ts, introspection_cpp::typesupport_identifier)) {
        view.template emplace<const CppMembers*>(static_cast<const CppMembers*>(handle->data));
    } else if (auto handle = get_message_typesupport_handle(ts, rosidl_typesupport_introspection_c__identifier)) {
        view.template emplace<const CMembers*>(static_cast<const CMembers*>(handle->data));
    }
    return view;
}

} // namespace

// Public API --------------------------------------------------------------

auto is_self_contained(const rosidl_message_type_support_t* type_support) -> bool {
    auto view = find_introspection(type_support);

    if (auto* member = view.get<const CppMembers*>()) {
        return is_self_contained_impl(*member);
    }
    if (auto* member = view.get<const CMembers*>()) {
        return is_self_contained_impl(*member);
    }

    return false;
}

auto message_size(const rosidl_message_type_support_t* type_support) -> size_t {
    auto view = find_introspection(type_support);

    if (auto* member = view.get<const CppMembers*>()) {
        return (*member)->size_of_;
    }
    if (auto* member = view.get<const CMembers*>()) {
        return (*member)->size_of_;
    }

    RMW_IOX2_CHAIN_ERROR_MSG("failed to determine message size");
    return 0;
}

auto message_type_name(const rosidl_message_type_support_t* type_support) -> std::string {
    auto view = find_introspection(type_support);

    const char* type_namespace = nullptr;
    const char* name = nullptr;
    if (auto* member = view.get<const CppMembers*>()) {
        type_namespace = (*member)->message_namespace_;
        name = (*member)->message_name_;
    } else if (auto* member = view.get<const CMembers*>()) {
        type_namespace = (*member)->message_namespace_;
        name = (*member)->message_name_;
    } else {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to determine message type name");
        return {};
    }

    // The namespace separator is `__` (C typesupport) or `::` (C++ typesupport); the rosidl name
    // uses `/`. Normalize both separators and append the message name.
    std::string result(type_namespace);
    for (const auto* separator : {"__", "::"}) {
        for (auto pos = result.find(separator); pos != std::string::npos; pos = result.find(separator, pos + 1)) {
            result.replace(pos, 2, "/");
        }
    }
    result += '/';
    result += name;

    return result;
}

auto message_type_hash(const rosidl_message_type_support_t* type_support) -> ::iox2::bb::Optional<rosidl_type_hash_t> {
    if (type_support == nullptr || type_support->get_type_hash_func == nullptr) {
        return ::iox2::bb::NULLOPT;
    }
    const rosidl_type_hash_t* hash = type_support->get_type_hash_func(type_support);
    if (hash == nullptr) {
        return ::iox2::bb::NULLOPT;
    }
    return *hash;
}

auto serialized_message_size(const void* ros_message, const rosidl_message_type_support_t* type_support) -> size_t {
    if (!type_support || !type_support->data) {
        return 0;
    }
    // Both the C++ and C fastrtps typesupports store the same
    // `message_type_support_callbacks_t`, so try either identifier; the C
    // variant is used e.g. by rcl's `/rosout` logging publisher.
    auto handle = get_handle(type_support, RMW_ICEORYX2_CXX_TYPESUPPORT_CPP);
    if (!handle) {
        handle = get_handle(type_support, RMW_ICEORYX2_CXX_TYPESUPPORT_C);
    }
    if (handle) {
        auto callbacks = static_cast<const message_type_support_callbacks_t*>(handle->data);
        return 4 + callbacks->get_serialized_size(ros_message); // 4 bytes for CDR header
    }
    return 0;
}

} // namespace rmw::iox2
