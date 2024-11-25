// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

namespace rmw::iox2
{

NodeImpl::NodeImpl(
    CreationLock, iox::optional<ErrorType>& error, uint32_t context_id, uint32_t node_id, const std::string& name)
    : m_name{name} {
    using ::rmw::iox2::create_in_place;

    auto iox2_name = ::iox2::NodeName::create(name.c_str());
    if (iox2_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_name.error()));
        error.emplace(ErrorType::NODE_NAME_CREATION_FAILURE);
        return;
    }

    auto iox2_node = ::iox2::NodeBuilder().name(iox2_name.value()).create<::iox2::ServiceType::Ipc>();
    if (iox2_node.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_node.error()));
        error.emplace(ErrorType::NODE_CREATION_FAILURE);
        return;
    }
    m_iox2.emplace(std::move(iox2_node.value()));

    if (auto result =
            create_in_place<GuardConditionImpl>(m_graph_guard_condition, std::ref(*this), context_id, node_id);
        result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create GuardConditionImpl for ContextImpl");
        error.emplace(ErrorType::GRAPH_GUARD_CONDITION_CREATION_FAILURE);
        return;
    }
}

auto NodeImpl::name() const -> const std::string& {
    return m_name;
}

auto NodeImpl::as_iox2() -> IceoryxNode& {
    return m_iox2.value();
}

auto NodeImpl::graph_guard_condition() -> GuardConditionImpl& {
    return m_graph_guard_condition.value();
}

} // namespace rmw::iox2
