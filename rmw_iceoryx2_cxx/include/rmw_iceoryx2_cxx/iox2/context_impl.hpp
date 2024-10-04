// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_CONTEXT_IMPL_HPP_
#define RMW_IOX2_CONTEXT_IMPL_HPP_

#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

#include <atomic>

extern "C" {

/// RMW expects exactly this type to be defined for context implementations
class RMW_PUBLIC rmw_context_impl_s
{
    using NodeImpl = ::rmw::iox2::NodeImpl;

public:
    explicit rmw_context_impl_s(const uint32_t id);

    auto context_id() -> uint32_t;
    auto next_guard_condition_id() -> uint32_t;
    auto node() -> NodeImpl&;

private:
    const uint32_t m_id;
    NodeImpl m_node;
    std::atomic<uint32_t> m_guard_condition_counter{0};
};
}

namespace rmw::iox2
{

// Used throughout implementation to indicate it is C++
using ContextImpl = rmw_context_impl_s;

} // namespace rmw::iox2

#endif
