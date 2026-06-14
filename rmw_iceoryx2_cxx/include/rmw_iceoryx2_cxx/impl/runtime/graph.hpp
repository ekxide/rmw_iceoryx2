// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_GRAPH_HPP_
#define RMW_IOX2_RUNTIME_GRAPH_HPP_

#include "iox2/bb/expected.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"

#include <functional>
#include <string>
#include <tuple>
#include <vector>

namespace rmw::iox2
{

class Graph;

template <>
struct Error<Graph>
{
    using Type = GraphError;
};

/// @brief A discovered node's name and namespace.
struct NodeName
{
    std::string name;
    std::string ns;

    auto operator<(const NodeName& other) const -> bool {
        return std::tie(ns, name) < std::tie(other.ns, other.name);
    }
};

/// @brief A discovered topic and its type.
struct TopicInfo
{
    std::string name;
    std::string type;

    auto operator<(const TopicInfo& other) const -> bool {
        return std::tie(name, type) < std::tie(other.name, other.type);
    }
};

/// @brief Read-only view over the iceoryx2 communication graph.
/// @details Provides discovery of nodes, topics, endpoints and their types and
///          QoS by introspecting the iceoryx2 service registry.
///
/// It owns no iceoryx2 entity lifetime, so it is a lightweight reference wrapper
/// rather than a `CreationLock`/`create()`-constructed type.
class RMW_PUBLIC Graph
{
public:
    using ErrorType = Error<Graph>::Type;

public:
    /// @brief Construct a graph view bound to a node's iceoryx2 instance.
    /// @param[in] node The node whose iceoryx2 instance is used for lookups.
    explicit Graph(Node& node);

    /// @brief List all discoverable nodes in the graph.
    /// @return The name and namespace of each node, deduplicated and ordered,
    ///         or an error if the iceoryx2 registry cannot be read.
    auto node_names() -> ::iox2::bb::Expected<std::vector<NodeName>, ErrorType>;

    /// @brief List all discoverable topics and their types.
    /// @return The name and type of each topic, deduplicated and ordered, or an
    ///         error if the iceoryx2 registry cannot be read.
    auto topic_names_and_types() -> ::iox2::bb::Expected<std::vector<TopicInfo>, ErrorType>;

private:
    std::reference_wrapper<Node> m_node;
};

} // namespace rmw::iox2

#endif // RMW_IOX2_RUNTIME_GRAPH_HPP_
