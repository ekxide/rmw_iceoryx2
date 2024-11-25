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

#include "iox/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

#include <atomic>

class rmw_context_impl_s;

namespace rmw::iox2
{

template <>
struct Error<rmw_context_impl_s>
{
    using Type = ContextError;
};

} // namespace rmw::iox2

extern "C" {

/// RMW expects exactly this type to be defined for context implementations
class RMW_PUBLIC rmw_context_impl_s
{
    using CreationLock = ::rmw::iox2::CreationLock;
    using NodeImpl = ::rmw::iox2::NodeImpl;
    using GuardConditionImpl = ::rmw::iox2::GuardConditionImpl;

public:
    using ErrorType = ::rmw::iox2::Error<rmw_context_impl_s>::Type;

public:
    rmw_context_impl_s(CreationLock, iox::optional<ErrorType>& error, const uint32_t id);

    auto id() -> uint32_t;
    auto generate_node_id() -> uint32_t;
    auto node() -> NodeImpl&;

private:
    const uint32_t m_id;
    iox::optional<NodeImpl> m_node;
    std::atomic<uint32_t> m_node_counter{0};
};
}

namespace rmw::iox2
{

// Used throughout implementation to indicate it is C++
using ContextImpl = rmw_context_impl_s;

} // namespace rmw::iox2

#endif
