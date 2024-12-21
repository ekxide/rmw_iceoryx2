// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rcutils/strdup.h"
#include "rcutils/types/string_array.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"
#include "rmw_iceoryx2_cxx/allocator.hpp"
#include "rmw_iceoryx2_cxx/defaults.hpp"
#include "rmw_iceoryx2_cxx/ensure.hpp"
#include "rmw_iceoryx2_cxx/error_message.hpp"
#include "rmw_iceoryx2_cxx/iox2/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/iox2/names.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

#include <functional>
#include <set>
#include <string>
#include <string_view>

namespace
{

class NodeName
{
public:
    NodeName(std::string ns = "", std::string n = "")
        : m_namespace(std::move(ns))
        , m_name(std::move(n)) {
    }

    auto ns() const -> const std::string& {
        return m_namespace;
    }

    auto name() const -> const std::string& {
        return m_name;
    }

    auto operator<(const NodeName& other) const -> bool {
        if (m_namespace != other.m_namespace) {
            return m_namespace < other.m_namespace;
        }
        return m_name < other.m_name;
    }

    static auto parse_name(std::string_view full_name) -> iox::optional<NodeName> {
        // Check for prefix
        constexpr std::string_view ROS2_PREFIX = "ros2://context/";
        if (full_name.substr(0, ROS2_PREFIX.length()) != ROS2_PREFIX) {
            return iox::nullopt;
        }

        // Find the "/nodes/" part after the context ID
        constexpr std::string_view NODES_MARKER = "/nodes/";
        auto nodes_pos = full_name.find(NODES_MARKER);
        if (nodes_pos == std::string_view::npos) {
            return iox::nullopt;
        }

        // Extract the part after "/nodes/"
        auto node_part = full_name.substr(nodes_pos + NODES_MARKER.length());
        if (node_part.empty()) {
            return iox::nullopt;
        }

        // Split into namespace and name
        auto last_slash = node_part.find_last_of('/');
        if (last_slash == std::string_view::npos) {
            return NodeName("", std::string(node_part));
        }

        return NodeName(std::string(node_part.substr(0, last_slash)), std::string(node_part.substr(last_slash + 1)));
    }

private:
    std::string m_namespace;
    std::string m_name;
};

class TopicDetails
{
public:
    TopicDetails(const std::string& topic, const std::string& type)
        : m_topic{topic}
        , m_type{type} {
    }

    auto name() const -> const std::string& {
        return m_topic;
    }

    auto type() const -> const std::string& {
        return m_type;
    }

    auto operator<(const TopicDetails& other) const -> bool {
        if (m_topic != other.m_topic) {
            return m_topic < other.m_topic;
        }
        return m_type < other.m_type;
    }

    static auto parse_topic_name(const char* full_name) -> ::iox::optional<std::string> {
        // Check for prefix
        constexpr std::string_view ROS2_PREFIX = "ros2://topics";
        std::string_view full_view(full_name);
        if (full_view.substr(0, ROS2_PREFIX.length()) != ROS2_PREFIX) {
            return iox::nullopt;
        }

        // Extract the topic part after "ros2://topics"
        auto topic_part = full_view.substr(ROS2_PREFIX.length());
        if (topic_part.empty()) {
            return iox::nullopt;
        }

        return std::string(topic_part);
    }

private:
    std::string m_topic;
    std::string m_type;
};

/// @brief Initialize a string array with the given size using the provided allocator
/// @param[in,out] array The string array to initialize
/// @param[in] size The size to initialize the array with
/// @param[in] allocator The allocator to use for memory allocation
/// @return RMW_RET_OK if successful, otherwise an appropriate error code
static rmw_ret_t init_string_array(rcutils_string_array_t* array, size_t size, rcutils_allocator_t* allocator) {
    auto ret = rcutils_string_array_init(array, size, allocator);
    if (ret != RCUTILS_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG(rcutils_get_error_string().str);
        return rmw_convert_rcutils_ret_to_rmw_ret(ret);
    }
    return RMW_RET_OK;
}

rmw_ret_t collect_node_names(std::set<NodeName>& names) {
    using ::iox2::CallbackProgression;
    using ::rmw::iox2::Iceoryx2;

    rmw_ret_t result = RMW_RET_OK;

    Iceoryx2::InterProcess::Handle::list(Iceoryx2::Config::global_config(), [&names](auto node) {
        node.alive([&names](const auto view) {
            view.details().and_then([&names](const auto details) {
                if (auto node_name = NodeName::parse_name(details.name().to_string().c_str())) {
                    names.emplace(*node_name);
                }
            });
        });
        return CallbackProgression::Continue;
    }).or_else([&result](auto) { result = RMW_RET_ERROR; });

    return result;
}


rmw_ret_t collect_topic_names_and_types(
    std::set<TopicDetails>& topics, std::function<bool(::iox2::StaticConfig&)> predicate = [](auto) { return true; }) {
    using ::iox2::CallbackProgression;
    using ::iox2::MessagingPattern;
    using ::rmw::iox2::Iceoryx2;

    rmw_ret_t result = RMW_RET_OK;

    Iceoryx2::InterProcess::Service::list(Iceoryx2::Config::global_config(), [&topics, &predicate](auto service) {
        if (predicate(service.static_details)) {
            auto topic = TopicDetails::parse_topic_name(service.static_details.name());
            topics.emplace(*topic, "");
        }
        return CallbackProgression::Continue;
    }).or_else([&result](auto) { result = RMW_RET_ERROR; });

    return result;
}

} // namespace

extern "C" {

// Nodes ==========================================================================================================

rmw_ret_t rmw_get_node_names(const rmw_node_t* node,
                             rcutils_string_array_t* node_names,
                             rcutils_string_array_t* node_namespaces) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*node_names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*node_namespaces, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    std::set<NodeName> names{};
    auto result = collect_node_names(names);
    RMW_IOX2_ENSURE_OK(result);

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    result = init_string_array(node_names, names.size(), &allocator);
    if (result != RMW_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node names");
        return result;
    }
    result = init_string_array(node_namespaces, names.size(), &allocator);
    if (result != RMW_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for node namespaces");
        return result;
    }

    int i = 0;
    for (const auto& name : names) {
        node_names->data[i] = rcutils_strdup(name.name().c_str(), allocator);
        if (!node_names->data[i]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to populate node name array");
            return RMW_RET_BAD_ALLOC;
        }

        node_namespaces->data[i] = rcutils_strdup(name.ns().c_str(), allocator);
        if (!node_namespaces->data[i]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to populate node namespace array");
            return RMW_RET_BAD_ALLOC;
        }

        ++i;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*node_names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*node_namespaces, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(enclaves, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*enclaves, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

// Publishers ======================================================================================================

rmw_ret_t rmw_count_publishers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    using ::rmw::iox2::Iceoryx2;
    using ::rmw::iox2::NodeImpl;
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;
    namespace names = rmw::iox2::names;

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    auto node_impl_result = unsafe_cast<NodeImpl*>(node->data);
    if (node_impl_result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto service_name_string = names::topic(topic_name);
    auto iox2_service_name = Iceoryx2::ServiceName::create(service_name_string.c_str());
    if (iox2_service_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create service name");
        return RMW_RET_ERROR;
    }

    auto service_result = node_impl->iox2()
                              .ipc()
                              .service_builder(iox2_service_name.value())
                              .publish_subscribe<PublisherImpl::Payload>()
                              .open_or_create();
    if (service_result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to open service");
        return RMW_RET_ERROR;
    }
    auto& service = service_result.value();
    (void)service;

    // *count = service.dynamic_config().number_of_publishers(); // NOT IMPLEMENTED ...

    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace,
                                                    bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types) {
    using ::iox2::MessagingPattern;

    (void)no_demangle; // not used

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NAMESPACE(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(topic_names_and_types->names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*topic_names_and_types->types, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* node,
                                           rcutils_allocator_t* allocator,
                                           const char* topic_name,
                                           bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(publishers_info, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!rcutils_allocator_is_valid(allocator)) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if (rmw_topic_endpoint_info_array_check_zero(publishers_info) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

// Subscribers ======================================================================================================

rmw_ret_t rmw_count_subscribers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    using ::rmw::iox2::Iceoryx2;
    using ::rmw::iox2::NodeImpl;
    using ::rmw::iox2::SubscriberImpl;
    using ::rmw::iox2::unsafe_cast;
    namespace names = rmw::iox2::names;

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    auto node_impl_result = unsafe_cast<NodeImpl*>(node->data);
    if (node_impl_result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto service_name_string = names::topic(topic_name);
    auto iox2_service_name = Iceoryx2::ServiceName::create(service_name_string.c_str());
    if (iox2_service_name.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create service name");
        return RMW_RET_ERROR;
    }

    auto service_result = node_impl->iox2()
                              .ipc()
                              .service_builder(iox2_service_name.value())
                              .publish_subscribe<SubscriberImpl::Payload>()
                              .open_or_create();
    if (service_result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to open service");
        return RMW_RET_ERROR;
    }
    auto& service = service_result.value();
    (void)service;

    // *count = service.dynamic_config().number_of_subscribers(); // NOT_IMPLEMENTED...

    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace,
                                                     bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(topic_names_and_types->names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*topic_names_and_types->types, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name,
                                              bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(subscriptions_info, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!rcutils_allocator_is_valid(allocator)) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if (rmw_topic_endpoint_info_array_check_zero(subscriptions_info) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

// Topics ===========================================================================================================

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node,
                                        rcutils_allocator_t* allocator,
                                        bool no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types) {
    // Invariants -----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!rcutils_allocator_is_valid(allocator)) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    auto zero_array = rcutils_get_zero_initialized_string_array();
    auto comparison_result{0};
    if (rcutils_string_array_cmp(&topic_names_and_types->names, &zero_array, &comparison_result) != RMW_RET_OK
        || comparison_result != 0) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to verify topic names is zero initialized");
        return RMW_RET_INVALID_ARGUMENT;
    };
    if (rcutils_string_array_cmp(topic_names_and_types->types, &zero_array, &comparison_result) != RMW_RET_OK
        || comparison_result != 0) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to verify topic types is zero initialized");
        return RMW_RET_INVALID_ARGUMENT;
    };

    // Implementation -------------------------------------------------------------------------------
    using ::iox2::MessagingPattern;

    std::set<TopicDetails> topics{};
    auto collect_result = collect_topic_names_and_types(topics, [](auto& service_details) {
        return service_details.messaging_pattern() == MessagingPattern::PublishSubscribe;
    });
    RMW_IOX2_ENSURE_OK(collect_result);

    auto init_result = rmw_names_and_types_init(topic_names_and_types, topics.size(), allocator);
    RMW_IOX2_ENSURE_OK(init_result);

    if (rcutils_string_array_init(topic_names_and_types->types, topics.size(), allocator) != RMW_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic types");
        return RMW_RET_BAD_ALLOC;
    }

    size_t index = 0;
    for (const auto& topic : topics) {
        // Allocate and copy topic name
        topic_names_and_types->names.data[index] = rcutils_strdup(topic.name().c_str(), *allocator);
        if (!topic_names_and_types->names.data[index]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
            return RMW_RET_BAD_ALLOC;
        }

        // Allocate and copy type name
        topic_names_and_types->types->data[index] = rcutils_strdup("UNKNOWN", *allocator);
        if (!topic_names_and_types->types->data[index]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for type type");
            return RMW_RET_BAD_ALLOC;
        }

        ++index;
    }

    return RMW_RET_OK;
}

// Services ==========================================================================================================

rmw_ret_t rmw_count_services(const rmw_node_t* node, const char* service_name, size_t* count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_SERVICE_NAME(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* node,
                                          rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(service_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(service_names_and_types->names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*service_names_and_types->types, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name,
                                                  const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NAMESPACE(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(service_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(service_names_and_types->names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*service_names_and_types->types, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

// Clients ==========================================================================================================

rmw_ret_t rmw_count_clients(const rmw_node_t* node, const char* service_name, size_t* count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_SERVICE_NAME(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}


rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NODE_NAME(node_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_NAMESPACE(node_namespace, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(service_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(service_names_and_types->names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*service_names_and_types->types, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}
}
