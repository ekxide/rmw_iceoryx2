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
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"

#include <array>
#include <cstdint>
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

/// @brief A discovered endpoint (publisher or subscriber) on a topic.
struct EndpointInfo
{
    std::string node_name;
    std::string node_namespace;
    std::string topic_type;
    Qos qos;
    // The endpoint's iceoryx2 unique port id, used as the rmw gid. Its length
    // matches rmw's `RMW_GID_STORAGE_SIZE` (both 16).
    std::array<uint8_t, ::iox2::UNIQUE_PORT_ID_LENGTH> gid;
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

    /// @brief Count the publishers currently connected to a topic.
    /// @param[in] topic The ROS topic name.
    /// @return The number of publishers, `0` if no service exists for the topic,
    ///         or an error if an existing service cannot be opened.
    auto count_publishers(const std::string& topic) -> ::iox2::bb::Expected<size_t, ErrorType>;

    /// @brief Count the subscribers currently connected to a topic.
    /// @param[in] topic The ROS topic name.
    /// @return The number of subscribers, `0` if no service exists for the topic,
    ///         or an error if an existing service cannot be opened.
    auto count_subscribers(const std::string& topic) -> ::iox2::bb::Expected<size_t, ErrorType>;

    /// @brief Describe the publishers currently connected to a topic.
    /// @param[in] topic The ROS topic name.
    /// @return One `EndpointInfo` per publisher (empty if no service exists for
    ///         the topic), or an error if an existing service cannot be opened.
    auto publishers_info(const std::string& topic) -> ::iox2::bb::Expected<std::vector<EndpointInfo>, ErrorType>;

    /// @brief Describe the subscribers currently connected to a topic.
    /// @param[in] topic The ROS topic name.
    /// @return One `EndpointInfo` per subscriber (empty if no service exists for
    ///         the topic), or an error if an existing service cannot be opened.
    auto subscriptions_info(const std::string& topic) -> ::iox2::bb::Expected<std::vector<EndpointInfo>, ErrorType>;

private:
    enum class EndpointKind : uint8_t { PUBLISHER, SUBSCRIBER };

    /// Shared implementation of `count_publishers`/`count_subscribers`: opens the
    /// topic's existing service and reads the requested endpoint count from its
    /// dynamic config. The payload type details are read from the registry so the
    /// service can be opened without the original typesupport.
    auto count_endpoints(const std::string& topic, EndpointKind kind) -> ::iox2::bb::Expected<size_t, ErrorType>;

    /// Shared implementation of `publishers_info`/`subscriptions_info`: opens the
    /// topic's existing service and describes each connected endpoint of the
    /// requested kind, resolving node ids to names via the node registry.
    auto endpoints_info(const std::string& topic,
                        EndpointKind kind) -> ::iox2::bb::Expected<std::vector<EndpointInfo>, ErrorType>;

    // `reference_wrapper` so the class remains move-constructible. Cannot be
    // null by construction.
    std::reference_wrapper<Node> m_node;
};

} // namespace rmw::iox2

#endif // RMW_IOX2_RUNTIME_GRAPH_HPP_
