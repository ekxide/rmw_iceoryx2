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
#include "rmw_iceoryx2_cxx/iox2/iceoryx2.hpp"

namespace rmw::iox2
{

class NodeImpl;

template <>
struct Error<NodeImpl>
{
    using Type = NodeError;
};

/// @brief Implementation of the RMW node for iceoryx2
/// @details The node manages the lifetime of communication endpoints and implements the functionality to realize the
///          communication graph.
///
/// The graph data structure is managed by iceoryx2 directly, related operations can be achived via the handle to
/// iceoryx2.
///
class RMW_PUBLIC NodeImpl
{
public:
    using ErrorType = Error<NodeImpl>::Type;

public:
    /// @brief Constructor for the iceoryx2 implementation of an RMW node
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context which this node will belong to
    /// @param[in] name The name of the node
    /// @param[in] ns The namespace of the node
    NodeImpl(CreationLock, iox::optional<ErrorType>& error, ContextImpl& context, const char* name, const char* ns);

    /// @brief Get the name of the node
    /// @return The name of the node
    auto name() const -> const std::string&;

    /// @brief Get the handle to the underlying iceoryx runtime
    /// @return Reference to the iceoryx handle
    auto iox2() -> Iceoryx2&;

    /// @brief Get the guard condition for notifying of graph events
    /// @return The guard condition for graph events
    auto graph_guard_condition() -> GuardConditionImpl&;

private:
    const std::string m_name;
    iox::optional<Iceoryx2> m_iox2;
    iox::optional<GuardConditionImpl> m_graph_guard_condition;
};

} // namespace rmw::iox2

#endif
