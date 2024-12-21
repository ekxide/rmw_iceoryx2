// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"

namespace rmw::iox2
{

Iceoryx2::Iceoryx2(CreationLock, iox::optional<ErrorType>& error, const std::string& instance_name) {
    auto iox2_name = Iceoryx2::InstanceName::create(instance_name.c_str());
    if (iox2_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(iox2_name.error()));
        error.emplace(ErrorType::INVALID_INSTANCE_NAME);
        return;
    }

    auto local_node = Iceoryx2::InstanceBuilder().name(iox2_name.value()).create<Iceoryx2::ServiceType::Local>();
    if (local_node.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(local_node.error()));
        error.emplace(ErrorType::ICEORYX_HANDLE_CREATION_FAILURE);
        return;
    }
    m_local.emplace(std::move(local_node.value()));

    auto ipc_node = Iceoryx2::InstanceBuilder().name(iox2_name.value()).create<Iceoryx2::ServiceType::Ipc>();
    if (ipc_node.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(ipc_node.error()));
        error.emplace(ErrorType::ICEORYX_HANDLE_CREATION_FAILURE);
        return;
    }
    m_ipc.emplace(std::move(ipc_node.value()));
};

auto Iceoryx2::local() -> Local::Handle& {
    if (!m_local.has_value()) {
        IOX_PANIC("Invalid access to iceoryx2 instance");
    }
    return m_local.value();
}

auto Iceoryx2::ipc() -> InterProcess::Handle& {
    if (!m_ipc.has_value()) {
        IOX_PANIC("Invalid access to iceoryx2 instance");
    }
    return m_ipc.value();
}

} // namespace rmw::iox2
