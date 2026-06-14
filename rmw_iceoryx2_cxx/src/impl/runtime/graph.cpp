// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/graph.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/message_type_details.hpp"
#include "iox2/service.hpp"
#include "iox2/service_builder_publish_subscribe.hpp"
#include "iox2/static_config.hpp"
#include "iox2/unique_node_id.hpp"
#include "rmw_iceoryx2_cxx/impl/common/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"
#include "rosidl_runtime_c/type_hash.h"

#include <algorithm>
#include <map>
#include <set>
#include <string_view>
#include <utility>

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

/// A node's unique id as a comparable key: (high bits, low bits).
using NodeIdKey = std::pair<uint64_t, uint64_t>;

auto to_key(const ::iox2::UniqueNodeId& id) -> NodeIdKey {
    return {id.value_high(), id.value_low()};
}

/// Build a lookup, from node unique id to its parsed (name, namespace), by
/// inspecting the alive nodes in iceoryx2.
auto build_node_id_lookup(::rmw::iox2::Node& node) -> std::map<NodeIdKey, ::rmw::iox2::NodeName> {
    using ::iox2::CallbackProgression;
    using ::rmw::iox2::Iceoryx2;

    std::map<NodeIdKey, ::rmw::iox2::NodeName> nodes{};
    auto config = node.iox2().ipc().config();
    [[maybe_unused]] auto list_result = Iceoryx2::InterProcess::Handle::list(config, [&nodes](auto node_state) {
        node_state.alive([&nodes](const auto view) {
            const auto& details = view.details();
            if (details.has_value()) {
                auto name_str = details.value().name().to_string();
                if (auto node_name = parse_node_name(name_str.unchecked_access().c_str()); node_name.has_value()) {
                    nodes.emplace(to_key(view.id()), std::move(node_name.value()));
                }
            }
        });
        return CallbackProgression::Continue;
    });
    return nodes;
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
                // The rmw stores the ROS type name (`<pkg>/msg/<Type>`) as the
                // iceoryx2 payload type name at publisher/subscriber creation.
                auto payload = service.static_details.publish_subscribe().message_type_details().payload();
                std::string type = payload.type_name();
                topics.emplace(TopicInfo{std::move(topic.value()), type.empty() ? "UNKNOWN" : std::move(type)});
            }
        }
        return CallbackProgression::Continue;
    });
    if (!list_result.has_value()) {
        return err(ErrorType::LISTING_FAILURE);
    }

    return std::vector<TopicInfo>{topics.begin(), topics.end()};
}

auto Graph::count_publishers(const std::string& topic) -> ::iox2::bb::Expected<size_t, ErrorType> {
    return count_endpoints(topic, EndpointKind::PUBLISHER);
}

auto Graph::count_subscribers(const std::string& topic) -> ::iox2::bb::Expected<size_t, ErrorType> {
    return count_endpoints(topic, EndpointKind::SUBSCRIBER);
}

auto Graph::count_endpoints(const std::string& topic, EndpointKind kind) -> ::iox2::bb::Expected<size_t, ErrorType> {
    using ::iox2::bb::err;
    using Payload = ::rmw::iox2::Publisher::Payload;
    using UserHeader = ::rmw::iox2::Publisher::UserHeader;
    namespace names = ::rmw::iox2::names;

    auto& node = m_node.get();
    auto service_name = names::topic(topic.c_str());

    // A topic with no service simply has no endpoints. Reading the existing
    // service's payload type details from the registry lets us open it (to reach
    // its dynamic config) without the original typesupport.
    auto details = node.iox2().lookup_service<Iceoryx2::ServiceType::Ipc>(service_name,
                                                                          Iceoryx2::MessagingPattern::PublishSubscribe);
    if (!details.has_value()) {
        return size_t{0};
    }
    auto payload_type_details = details.value().static_details.publish_subscribe().message_type_details().payload();

    auto iox2_service_name = Iceoryx2::ServiceName::create(service_name.c_str());
    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        return err(ErrorType::SERVICE_NAME_CREATION_FAILURE);
    }

    auto service_builder = node.iox2()
                               .ipc()
                               .service_builder(iox2_service_name.value())
                               .publish_subscribe<Payload>()
                               .user_header<UserHeader>();
    ::iox2::set_payload_type_details(service_builder, payload_type_details);

    auto service = service_builder.resume_build().open();
    if (!service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(service.error()));
        return err(ErrorType::SERVICE_OPEN_FAILURE);
    }

    const auto& dynamic_config = service.value().dynamic_config();

    return kind == EndpointKind::PUBLISHER ? dynamic_config.number_of_publishers()
                                           : dynamic_config.number_of_subscribers();
}

auto Graph::publishers_info(const std::string& topic) -> ::iox2::bb::Expected<std::vector<EndpointInfo>, ErrorType> {
    return endpoints_info(topic, EndpointKind::PUBLISHER);
}

auto Graph::subscriptions_info(const std::string& topic) -> ::iox2::bb::Expected<std::vector<EndpointInfo>, ErrorType> {
    return endpoints_info(topic, EndpointKind::SUBSCRIBER);
}

auto Graph::endpoints_info(const std::string& topic,
                           EndpointKind kind) -> ::iox2::bb::Expected<std::vector<EndpointInfo>, ErrorType> {
    using ::iox2::CallbackProgression;
    using ::iox2::bb::err;
    using Payload = ::rmw::iox2::Publisher::Payload;
    using UserHeader = ::rmw::iox2::Publisher::UserHeader;
    namespace names = ::rmw::iox2::names;

    auto& node = m_node.get();
    auto service_name = names::topic(topic.c_str());

    auto details = node.iox2().lookup_service<Iceoryx2::ServiceType::Ipc>(service_name,
                                                                          Iceoryx2::MessagingPattern::PublishSubscribe);
    if (!details.has_value()) {
        // A topic with no service has no endpoints.
        return std::vector<EndpointInfo>{};
    }
    // The payload type details let us open the service without reading
    // from the typesupport.
    auto payload_type_details = details.value().static_details.publish_subscribe().message_type_details().payload();
    std::string topic_type = payload_type_details.type_name();
    if (topic_type.empty()) {
        topic_type = "UNKNOWN";
    }

    auto iox2_service_name = Iceoryx2::ServiceName::create(service_name.c_str());
    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        return err(ErrorType::SERVICE_NAME_CREATION_FAILURE);
    }

    auto service_builder = node.iox2()
                               .ipc()
                               .service_builder(iox2_service_name.value())
                               .publish_subscribe<Payload>()
                               .user_header<UserHeader>();

    ::iox2::set_payload_type_details(service_builder, payload_type_details);

    auto service = service_builder.resume_build().open();
    if (!service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(service.error()));
        return err(ErrorType::SERVICE_OPEN_FAILURE);
    }
    auto& port_factory = service.value();

    // All endpoints on a topic share the service-level QoS and type hash in
    // iceoryx2.
    auto qos = TryConvert<Qos>::from(port_factory.attributes(), ProfileKind::PUBLISH_SUBSCRIBE);
    if (!qos.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to decode QoS from service attributes");
        return err(ErrorType::QOS_DECODING_FAILURE);
    }

    auto type_hash = rosidl_get_zero_initialized_type_hash();
    attributes::visit_attribute_value(port_factory.attributes(), attributes::TypeHash::KEY, [&](const char* value) {
        if (auto decoded = attributes::TypeHash::decode(value); decoded.has_value()) {
            type_hash = decoded.value();
        }
    });

    auto nodes = build_node_id_lookup(node);

    // Assemble one endpoint's info: resolve its node id to a name/namespace, use
    // its iceoryx2 unique port id as the gid, and attach the shared topic type,
    // type hash, and service-level QoS.
    auto endpoint_info = [&](const ::iox2::UniqueNodeId& node_id,
                             const ::iox2::bb::Optional<::iox2::RawIdType>& gid_bytes) -> EndpointInfo {
        std::string node_name{};
        std::string node_namespace{};
        if (auto it = nodes.find(to_key(node_id)); it != nodes.end()) {
            node_name = it->second.name;
            node_namespace = it->second.ns;
        }

        std::array<uint8_t, ::iox2::UNIQUE_PORT_ID_LENGTH> gid{};
        if (gid_bytes.has_value()) {
            const auto& raw = gid_bytes.value();
            std::copy(raw.unchecked_access().begin(), raw.unchecked_access().end(), gid.begin());
        }

        return EndpointInfo{std::move(node_name), std::move(node_namespace), topic_type, type_hash, qos.value(), gid};
    };

    std::vector<EndpointInfo> result{};
    if (kind == EndpointKind::PUBLISHER) {
        port_factory.dynamic_config().list_publishers([&result, &endpoint_info](auto view) {
            result.push_back(endpoint_info(view.node_id(), view.publisher_id().bytes()));
            return CallbackProgression::Continue;
        });
    } else {
        port_factory.dynamic_config().list_subscribers([&result, &endpoint_info](auto view) {
            result.push_back(endpoint_info(view.node_id(), view.subscriber_id().bytes()));
            return CallbackProgression::Continue;
        });
    }

    return result;
}

} // namespace rmw::iox2
