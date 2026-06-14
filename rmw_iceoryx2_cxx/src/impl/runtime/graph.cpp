// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/graph.hpp"

#include "iox2/bb/optional.hpp"
#include "iox2/service.hpp"
#include "iox2/static_config.hpp"

#include <set>
#include <string_view>

namespace
{

/// Parse a node's namespace and name out of an iceoryx2 instance name of the
/// form `ros2://context/<id>/nodes/<namespace>/<name>`.
auto parse_node_name(std::string_view full_name) -> ::iox2::bb::Optional<::rmw::iox2::NodeName> {
    using ::rmw::iox2::NodeName;

    constexpr std::string_view ROS2_PREFIX = "ros2://context/";
    if (full_name.substr(0, ROS2_PREFIX.length()) != ROS2_PREFIX) {
        return ::iox2::bb::NULLOPT;
    }

    // Find the "/nodes/" part after the context ID.
    constexpr std::string_view NODES_MARKER = "/nodes/";
    auto nodes_pos = full_name.find(NODES_MARKER);
    if (nodes_pos == std::string_view::npos) {
        return ::iox2::bb::NULLOPT;
    }

    // Extract the part after "/nodes/".
    auto node_part = full_name.substr(nodes_pos + NODES_MARKER.length());
    if (node_part.empty()) {
        return ::iox2::bb::NULLOPT;
    }

    // Split into namespace and name.
    auto last_slash = node_part.find_last_of('/');
    if (last_slash == std::string_view::npos) {
        return NodeName{std::string(node_part), ""};
    }

    return NodeName{std::string(node_part.substr(last_slash + 1)), std::string(node_part.substr(0, last_slash))};
}

/// Parse a ROS topic name out of an iceoryx2 service name of the form
/// `ros2://topics<topic>`.
auto parse_topic_name(const char* full_name) -> ::iox2::bb::Optional<std::string> {
    constexpr std::string_view ROS2_PREFIX = "ros2://topics";
    std::string_view full_view(full_name);
    if (full_view.substr(0, ROS2_PREFIX.length()) != ROS2_PREFIX) {
        return ::iox2::bb::NULLOPT;
    }

    auto topic_part = full_view.substr(ROS2_PREFIX.length());
    if (topic_part.empty()) {
        return ::iox2::bb::NULLOPT;
    }

    return std::string(topic_part);
}

} // namespace

namespace rmw::iox2
{

Graph::Graph(Node& node)
    : m_node{node} {
}

auto Graph::node_names() -> ::iox2::bb::Expected<std::vector<NodeName>, ErrorType> {
    using ::iox2::CallbackProgression;
    using ::iox2::bb::err;

    std::set<NodeName> names{};
    auto config = m_node.get().iox2().ipc().config();
    auto list_result = Iceoryx2::InterProcess::Handle::list(config, [&names](auto node_state) {
        node_state.alive([&names](const auto view) {
            const auto& details = view.details();
            if (details.has_value()) {
                auto name_str = details.value().name().to_string();
                if (auto node_name = parse_node_name(name_str.unchecked_access().c_str()); node_name.has_value()) {
                    names.emplace(std::move(node_name.value()));
                }
            }
        });
        return CallbackProgression::Continue;
    });
    if (!list_result.has_value()) {
        return err(ErrorType::LISTING_FAILURE);
    }

    return std::vector<NodeName>{names.begin(), names.end()};
}

auto Graph::topic_names_and_types() -> ::iox2::bb::Expected<std::vector<TopicInfo>, ErrorType> {
    using ::iox2::CallbackProgression;
    using ::iox2::MessagingPattern;
    using ::iox2::bb::err;

    std::set<TopicInfo> topics{};
    auto config = m_node.get().iox2().ipc().config();
    auto list_result = Iceoryx2::InterProcess::Service::list(config, [&topics](auto service) {
        if (service.static_details.messaging_pattern() == MessagingPattern::PublishSubscribe) {
            if (auto topic = parse_topic_name(service.static_details.name()); topic.has_value()) {
                // TODO: extract the real type from the service's message type
                //       details instead of the "UNKNOWN" placeholder.
                topics.emplace(TopicInfo{std::move(topic.value()), "UNKNOWN"});
            }
        }
        return CallbackProgression::Continue;
    });
    if (!list_result.has_value()) {
        return err(ErrorType::LISTING_FAILURE);
    }

    return std::vector<TopicInfo>{topics.begin(), topics.end()};
}

} // namespace rmw::iox2
