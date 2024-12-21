// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"

#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"

rmw_context_impl_s::rmw_context_impl_s(CreationLock, iox::optional<ErrorType>& error, const uint32_t id)
    : m_id{id} {
    using ::rmw::iox2::create_in_place;
    namespace names = rmw::iox2::names;

    if (auto result = create_in_place<Iceoryx2>(m_iox2, names::context(id)); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create Handle");
        error.emplace(ErrorType::HANDLE_CREATION_FAILURE);
        return;
    }
}

auto rmw_context_impl_s::id() -> uint32_t {
    return m_id;
}

auto rmw_context_impl_s::iox2() -> Iceoryx2& {
    return m_iox2.value();
}

auto rmw_context_impl_s::generate_guard_condition_id() -> uint32_t {
    return m_guard_condition_counter++;
}
