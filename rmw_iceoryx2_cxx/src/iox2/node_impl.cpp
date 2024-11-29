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
#include "rmw_iceoryx2_cxx/error_message.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"

namespace rmw::iox2
{

NodeImpl::NodeImpl(
    CreationLock, iox::optional<ErrorType>& error, ContextImpl& context, const char* name, const char* ns)
    : m_name{name} {
    using ::rmw::iox2::create_in_place;
    namespace names = rmw::iox2::names;

    if (auto result = create_in_place<Iceoryx2>(m_iox2, names::node(context.id(), name, ns)); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create Handle");
        error.emplace(ErrorType::HANDLE_CREATION_FAILURE);
        return;
    }

    if (auto result = create_in_place<GuardConditionImpl>(m_graph_guard_condition, context); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create GuardConditionImpl");
        error.emplace(ErrorType::GRAPH_GUARD_CONDITION_CREATION_FAILURE);
        return;
    }
}

auto NodeImpl::name() const -> const std::string& {
    return m_name;
}

auto NodeImpl::iox2() -> Iceoryx2& {
    return m_iox2.value();
}

auto NodeImpl::graph_guard_condition() -> GuardConditionImpl& {
    return m_graph_guard_condition.value();
}

} // namespace rmw::iox2
