// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_NODE_HPP_
#define RMW_IOX2_RUNTIME_NODE_HPP_

#include "iox2/bb/optional.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"

#include <functional>

namespace rmw::iox2
{

class Node;

template <>
struct Error<Node>
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
class RMW_PUBLIC Node
{
public:
    using ErrorType = Error<Node>::Type;

public:
    /// @brief Constructor for the iceoryx2 implementation of an RMW node
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context which this node will belong to
    /// @param[in] name The name of the node
    /// @param[in] ns The namespace of the node
    Node(CreationLock, ::iox2::bb::Optional<ErrorType>& error, Context& context, const char* name, const char* ns);

    /// @brief Get the name of the node
    /// @return The name of the node
    auto name() const -> const std::string&;

    /// @brief Get the handle to the underlying iceoryx runtime
    /// @return Reference to the iceoryx handle
    auto iox2() -> Iceoryx2&;

    /// @brief Get the owning context
    /// @return Reference to the context
    auto context() -> Context&;

    /// @brief Get the guard condition for notifying of graph events
    /// @return The guard condition for graph events
    auto graph_guard_condition() -> GuardCondition&;

private:
    // `reference_wrapper` so the class remains move-constructible, which
    // iox2::bb::Optional's emplace path requires. Cannot be null by construction.
    std::reference_wrapper<Context> m_context;
    std::string m_name;
    ::iox2::bb::Optional<Iceoryx2> m_iox2;
    ::iox2::bb::Optional<GuardCondition> m_graph_guard_condition;
};

} // namespace rmw::iox2

#endif
