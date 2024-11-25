// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_NODE_IMPL_HPP_
#define RMW_IOX2_NODE_IMPL_HPP_

#include "iox/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/handle.hpp"

namespace rmw::iox2
{

class NodeImpl;

template <>
struct Error<NodeImpl>
{
    using Type = NodeError;
};

class RMW_PUBLIC NodeImpl
{
public:
    using ErrorType = Error<NodeImpl>::Type;

public:
    NodeImpl(CreationLock, iox::optional<ErrorType>& error, ContextImpl& context, const char* name, const char* ns);

    auto name() const -> const std::string&;
    auto iox2() -> IceoryxHandle&;
    auto graph_guard_condition() -> GuardConditionImpl&;

private:
    const std::string m_name;
    iox::optional<IceoryxHandle> m_handle;
    iox::optional<GuardConditionImpl> m_graph_guard_condition;
};

} // namespace rmw::iox2

#endif
