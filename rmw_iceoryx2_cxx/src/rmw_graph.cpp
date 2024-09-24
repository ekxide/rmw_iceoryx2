// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"

extern "C" {
rmw_ret_t
rmw_get_node_names(const rmw_node_t* node, rcutils_string_array_t* node_names, rcutils_string_array_t* node_namespaces)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_count_publishers(const rmw_node_t* node, const char* topic_name, size_t* count)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_count_subscribers(const rmw_node_t* node, const char* topic_name, size_t* count)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_count_clients(const rmw_node_t* node, const char* service_name, size_t* count)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_count_services(const rmw_node_t* node, const char* service_name, size_t* count)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* publisher, rmw_gid_t* gid)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_gid_for_client(const rmw_client_t* client, rmw_gid_t* gid)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t* gid1, const rmw_gid_t* gid2, bool* result)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_service_server_is_available(const rmw_node_t* node, const rmw_client_t* client, bool* is_available)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace,
                                                     bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace,
                                                    bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types)
{
    return RMW_RET_ERROR;
}


rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name,
                                                  const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node,
                                        rcutils_allocator_t* allocator,
                                        bool no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* node,
                                          rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* node,
                                           rcutils_allocator_t* allocator,
                                           const char* topic_name,
                                           bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info)
{
    return RMW_RET_ERROR;
}

rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name,
                                              bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info)
{
    return RMW_RET_ERROR;
}
}
