// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"

#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"

rmw_context_impl_s::rmw_context_impl_s(CreationLock, iox::optional<ErrorType>& error, const uint32_t id)
    : m_id{id} {
    using ::rmw::iox2::create_in_place;

    if (auto result = create_in_place<NodeImpl>(m_node, rmw::iox2::names::context(id).c_str()); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create NodeImpl for ContextImpl");
        error.emplace(ErrorType::NODE_CREATION_FAILURE);
        return;
    }

    if (auto result = create_in_place<GuardConditionImpl>(
            m_graph_guard_condition, this->node(), this->id(), this->next_guard_condition_id());
        result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create GuardConditionImpl for ContextImpl");
        error.emplace(ErrorType::GRAPH_GUARD_CONDITION_CREATION_FAILURE);
        return;
    }
}

auto rmw_context_impl_s::id() -> uint32_t {
    return m_id;
}

auto rmw_context_impl_s::next_guard_condition_id() -> uint32_t {
    return m_guard_condition_counter++;
}

auto rmw_context_impl_s::node() -> NodeImpl& {
    return m_node.value();
}

auto rmw_context_impl_s::graph_guard_condition() -> GuardConditionImpl& {
    return m_graph_guard_condition.value();
}
