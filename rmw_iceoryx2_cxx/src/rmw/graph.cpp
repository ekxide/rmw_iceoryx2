// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "iox2/node.hpp"
#include "iox2/service_type.hpp"
#include "rcutils/strdup.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

#include <set>

struct NodeName
{
    std::string namespace_{""};
    std::string name{""};

    bool operator<(const NodeName& other) const {
        if (namespace_ != other.namespace_) {
            return namespace_ < other.namespace_;
        }
        return name < other.name;
    }
};

const NodeName parse_node_name(const char* full_name) {
    NodeName result{};

    const char* last_slash = strrchr(full_name, '/');
    if (last_slash == nullptr) {
        // no slashes found, treat the whole string as the name
        result.name = full_name;
    } else {
        // extract parts
        result.namespace_ = std::string(full_name, last_slash - full_name);
        result.name = std::string(last_slash + 1);

        // remove leading slash from namespace, if namespace is present
        if (!result.namespace_.empty() && result.namespace_[0] == '/') {
            result.namespace_ = result.namespace_.substr(1);
        }
    }

    return result;
}

extern "C" {
rmw_ret_t rmw_get_node_names(const rmw_node_t* node,
                             rcutils_string_array_t* node_names,
                             rcutils_string_array_t* node_namespaces) {
    using iox2::ServiceType;
    using Node = iox2::Node<ServiceType::Ipc>;
    using iox2::CallbackProgression;
    using iox2::Config;
    using rmw::iox2::NodeImpl;

    rmw_ret_t result = RMW_RET_OK;

    // retrieve names from node
    std::set<NodeName> names{};
    Node::list(Config::global_config(), [&names](auto node) {
        node.alive([&names](const auto view) {
            view.details().and_then([&names](const auto details) {
                auto parts = parse_node_name(details.name().to_string().c_str());
                names.emplace(parts);
            });
        });
        return CallbackProgression::Continue;
    }).or_else([&result](auto) { result = RMW_RET_ERROR; });
    RMW_IOX2_OK_OR_RETURN(result);

    // allocate output
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rcutils_ret_t rcutils_ret = rcutils_string_array_init(node_names, names.size(), &allocator);
    if (rcutils_ret != RCUTILS_RET_OK) {
        RMW_IOX2_SET_ERROR_MSG(rcutils_get_error_string().str);
        result = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        return result;
    }
    rcutils_ret = rcutils_string_array_init(node_namespaces, names.size(), &allocator);
    if (rcutils_ret != RCUTILS_RET_OK) {
        RMW_IOX2_SET_ERROR_MSG(rcutils_get_error_string().str);
        result = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        return result;
    }

    // set output
    int i = 0;
    for (auto name : names) {
        node_names->data[i] = rcutils_strdup(name.name.c_str(), allocator);
        if (!node_names->data[i]) {
            RMW_IOX2_SET_ERROR_MSG("could not allocate memory for node name");
            result = RMW_RET_ERROR;
            return result;
        }
        node_namespaces->data[i] = rcutils_strdup(name.namespace_.c_str(), allocator);
        if (!node_namespaces->data[i]) {
            RMW_IOX2_SET_ERROR_MSG("could not allocate memory for node namespace");
            result = RMW_RET_ERROR;
            return result;
        }
        ++i;
    }

    return result;
}

rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves) {
    IOX_TODO();
}

rmw_ret_t rmw_count_publishers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    IOX_TODO();
}

rmw_ret_t rmw_count_subscribers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    IOX_TODO();
}

rmw_ret_t rmw_count_clients(const rmw_node_t* node, const char* service_name, size_t* count) {
    IOX_TODO();
}

rmw_ret_t rmw_count_services(const rmw_node_t* node, const char* service_name, size_t* count) {
    IOX_TODO();
}

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* publisher, rmw_gid_t* gid) {
    IOX_TODO();
}

rmw_ret_t rmw_get_gid_for_client(const rmw_client_t* client, rmw_gid_t* gid) {
    IOX_TODO();
}

rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t* gid1, const rmw_gid_t* gid2, bool* result) {
    IOX_TODO();
}

rmw_ret_t rmw_service_server_is_available(const rmw_node_t* node, const rmw_client_t* client, bool* is_available) {
    IOX_TODO();
}

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace,
                                                     bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types) {
    IOX_TODO();
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace,
                                                    bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types) {
    IOX_TODO();
}


rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name,
                                                  const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types) {
    IOX_TODO();
}

rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types) {
    IOX_TODO();
}

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node,
                                        rcutils_allocator_t* allocator,
                                        bool no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types) {
    IOX_TODO();
}

rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* node,
                                          rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types) {
    IOX_TODO();
}

rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* node,
                                           rcutils_allocator_t* allocator,
                                           const char* topic_name,
                                           bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info) {
    IOX_TODO();
}

rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name,
                                              bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info) {
    IOX_TODO();
}
}
