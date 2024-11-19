// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

namespace rmw::iox2
{

NodeImpl::NodeImpl(CreationLock, iox::optional<ErrorType>& error, const std::string& name)
    : m_node_name{name} {
    auto node_name = ::iox2::NodeName::create(name.c_str());
    if (node_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(node_name.error()));
        error.emplace(ErrorType::NODE_NAME_CREATION_FAILURE);
        return;
    }

    auto node = ::iox2::NodeBuilder().name(node_name.value()).create<::iox2::ServiceType::Ipc>();
    if (node.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(node.error()));
        error.emplace(ErrorType::NODE_CREATION_FAILURE);
        return;
    }
    m_node.emplace(std::move(node.value()));
}

auto NodeImpl::node_name() const -> const std::string& {
    return m_node_name;
}

auto NodeImpl::as_iox2() -> IceoryxNode& {
    return m_node.value();
}

} // namespace rmw::iox2
