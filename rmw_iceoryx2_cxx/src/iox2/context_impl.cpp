// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"

#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/service_names.hpp"

// TODO: fallable constructors
//       make underlying type an optional, set RMW error on failure
rmw_context_impl_s::rmw_context_impl_s(const uint32_t id)
    : m_id{id}
    , m_node{NodeImpl(rmw::iox2::context_node_name(id).c_str())} {
}

uint32_t rmw_context_impl_s::id() {
    return m_id;
}

rmw_context_impl_s::GuardConditionImpl* rmw_context_impl_s::create_guard_condition() {
    using iox2::ServiceName;

    auto result = rmw::iox2::allocate<GuardConditionImpl>();
    if (!result) {
        RMW_IOX2_SET_ERROR_MSG("failed to allocate memory for guard condition impl");
        return result;
    }

    auto guard_condition_id = next_guard_condition_id();
    auto notifier = m_node.create_notifier(rmw::iox2::guard_condition_service_name(m_id, guard_condition_id).c_str());
    new (result) GuardConditionImpl(guard_condition_id, std::move(notifier));

    return result;
}

uint32_t rmw_context_impl_s::next_guard_condition_id() {
    return m_guard_condition_counter++;
}
