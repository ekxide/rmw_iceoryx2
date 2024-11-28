// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rcutils/strdup.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/iceoryx2.hpp"

#include <set>
#include <string>
#include <string_view>

class NodeName
{
public:
    NodeName(std::string ns = "", std::string n = "")
        : namespace_(std::move(ns))
        , name_(std::move(n)) {
    }

    const std::string& namespace_str() const {
        return namespace_;
    }
    const std::string& name_str() const {
        return name_;
    }

    bool operator<(const NodeName& other) const {
        if (namespace_ != other.namespace_) {
            return namespace_ < other.namespace_;
        }
        return name_ < other.name_;
    }

private:
    std::string namespace_;
    std::string name_;
};

/// @brief Parses the name and namespace from the name used to represent rmw nodes in iceoryx2
class NodeNameParser
{
public:
    static iox::optional<NodeName> parse(std::string_view full_name) {
        // Check for ROS2 prefix
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
};

/// @details Collects names of all rmw nodes present in iceoryx2
class NodeNameCollector
{
public:
    static rmw_ret_t collect_names(std::set<NodeName>& names) {
        using ::iox2::CallbackProgression;
        using ::rmw::iox2::Iceoryx2;

        rmw_ret_t result = RMW_RET_OK;

        Iceoryx2::InterProcess::Handle::list(Iceoryx2::Config::global_config(), [&names](auto node) {
            node.alive([&names](const auto view) {
                view.details().and_then([&names](const auto details) {
                    if (auto node_name = NodeNameParser::parse(details.name().to_string().c_str())) {
                        names.emplace(*node_name);
                    }
                });
            });
            return CallbackProgression::Continue;
        }).or_else([&result](auto) { result = RMW_RET_ERROR; });

        return result;
    }
};

/// @brief Helper to intialize strings via the rcutils allocator
class StringArrayInitializer
{
public:
    static rmw_ret_t init_arrays(rcutils_string_array_t* node_names,
                                 rcutils_string_array_t* node_namespaces,
                                 size_t size,
                                 const rcutils_allocator_t& allocator) {
        auto ret = rcutils_string_array_init(node_names, size, &allocator);
        if (ret != RCUTILS_RET_OK) {
            RMW_IOX2_CHAIN_ERROR_MSG(rcutils_get_error_string().str);
            return rmw_convert_rcutils_ret_to_rmw_ret(ret);
        }

        ret = rcutils_string_array_init(node_namespaces, size, &allocator);
        if (ret != RCUTILS_RET_OK) {
            RMW_IOX2_CHAIN_ERROR_MSG(rcutils_get_error_string().str);
            return rmw_convert_rcutils_ret_to_rmw_ret(ret);
        }

        return RMW_RET_OK;
    }
};

extern "C" {

rmw_ret_t rmw_get_node_names(const rmw_node_t* node,
                             rcutils_string_array_t* node_names,
                             rcutils_string_array_t* node_namespaces) {
    using iox2::ServiceType;

    std::set<NodeName> names{};
    auto result = NodeNameCollector::collect_names(names);
    RMW_IOX2_OK_OR_RETURN(result);

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    result = StringArrayInitializer::init_arrays(node_names, node_namespaces, names.size(), allocator);
    if (result != RMW_RET_OK) {
        return result;
    }

    int i = 0;
    for (const auto& name : names) {
        node_names->data[i] = rcutils_strdup(name.name_str().c_str(), allocator);
        if (!node_names->data[i]) {
            RMW_IOX2_CHAIN_ERROR_MSG("could not allocate memory for node name");
            return RMW_RET_ERROR;
        }

        node_namespaces->data[i] = rcutils_strdup(name.namespace_str().c_str(), allocator);
        if (!node_namespaces->data[i]) {
            RMW_IOX2_CHAIN_ERROR_MSG("could not allocate memory for node namespace");
            return RMW_RET_ERROR;
        }

        ++i;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_count_publishers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_count_subscribers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_count_clients(const rmw_node_t* node, const char* service_name, size_t* count) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_count_services(const rmw_node_t* node, const char* service_name, size_t* count) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_service_server_is_available(const rmw_node_t* node, const rmw_client_t* client, bool* is_available) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace,
                                                     bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace,
                                                    bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}


rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name,
                                                  const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node,
                                        rcutils_allocator_t* allocator,
                                        bool no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* node,
                                          rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* node,
                                           rcutils_allocator_t* allocator,
                                           const char* topic_name,
                                           bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name,
                                              bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info) {
    return RMW_RET_UNSUPPORTED;
}
}
