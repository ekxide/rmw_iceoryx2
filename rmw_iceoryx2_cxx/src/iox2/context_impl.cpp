// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"

#include "rmw_iceoryx2_cxx/iox2/names.hpp"

// TODO: fallable constructors
//       make underlying type an optional, set RMW error on failure
rmw_context_impl_s::rmw_context_impl_s(const uint32_t id)
    : m_id{id}
    , m_node{NodeImpl(rmw::iox2::names::context(id).c_str())} {
}

auto rmw_context_impl_s::id() -> uint32_t {
    return m_id;
}

auto rmw_context_impl_s::next_guard_condition_id() -> uint32_t {
    return m_guard_condition_counter++;
}

auto rmw_context_impl_s::node() -> NodeImpl& {
    return m_node;
}
