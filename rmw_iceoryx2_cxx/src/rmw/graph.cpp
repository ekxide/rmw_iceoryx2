// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/graph.hpp"
#include "rcutils/strdup.h"
#include "rcutils/types/string_array.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_endpoint_info.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw/validate_namespace.h"
#include "rmw/validate_node_name.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/attributes.hpp"
#include "rmw_iceoryx2_cxx/impl/common/defaults.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"

#include <vector>

namespace
{

// The rmw gid is filled from an iceoryx2 unique port id (see `EndpointInfo::gid`).
// Guard the cross-library contract here.
static_assert(::iox2::UNIQUE_PORT_ID_LENGTH == RMW_GID_STORAGE_SIZE,
              "iceoryx2 unique port id length must match RMW_GID_STORAGE_SIZE");

/// @brief Marshal discovered endpoints into an rmw endpoint-info array.
/// @param[out] array Zero-initialized array to populate.
/// @param[in] endpoints The endpoints to copy in.
/// @param[in] endpoint_type Whether these are publishers or subscriptions.
/// @param[in] allocator Allocator used for the array and its strings.
/// @return RMW_RET_OK on success, otherwise an appropriate error code.
static auto fill_endpoint_info_array(rmw_topic_endpoint_info_array_t* array,
                                     const std::vector<::rmw::iox2::EndpointInfo>& endpoints,
                                     rmw_endpoint_type_t endpoint_type,
                                     rcutils_allocator_t* allocator) -> rmw_ret_t {
    using ::rmw::iox2::Convert;

    if (rmw_topic_endpoint_info_array_init_with_size(array, endpoints.size(), allocator) != RMW_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG(rcutils_get_error_string().str);
        return RMW_RET_BAD_ALLOC;
    }

    for (size_t index = 0; index < endpoints.size(); ++index) {
        const auto& endpoint = endpoints[index];
        auto* info = &array->info_array[index];

        auto qos_profile = Convert<rmw_qos_profile_t>::from(endpoint.qos);
        if (rmw_topic_endpoint_info_set_node_name(info, endpoint.node_name.c_str(), allocator) != RMW_RET_OK
            || rmw_topic_endpoint_info_set_node_namespace(info, endpoint.node_namespace.c_str(), allocator)
                   != RMW_RET_OK
            || rmw_topic_endpoint_info_set_topic_type(info, endpoint.topic_type.c_str(), allocator) != RMW_RET_OK
            || rmw_topic_endpoint_info_set_topic_type_hash(info, &endpoint.type_hash) != RMW_RET_OK
            || rmw_topic_endpoint_info_set_endpoint_type(info, endpoint_type) != RMW_RET_OK
            || rmw_topic_endpoint_info_set_gid(info, endpoint.gid.data(), endpoint.gid.size()) != RMW_RET_OK
            || rmw_topic_endpoint_info_set_qos_profile(info, &qos_profile) != RMW_RET_OK) {
            RMW_IOX2_CHAIN_ERROR_MSG(rcutils_get_error_string().str);
            return RMW_RET_ERROR;
        }
    }

    return RMW_RET_OK;
}

/// @brief Initialize a string array with the given size using the provided allocator
/// @param[in,out] array The string array to initialize
/// @param[in] size The size to initialize the array with
/// @param[in] allocator The allocator to use for memory allocation
/// @return RMW_RET_OK if successful, otherwise an appropriate error code
static auto init_string_array(rcutils_string_array_t* array, size_t size, rcutils_allocator_t* allocator) -> rmw_ret_t {
    auto ret = rcutils_string_array_init(array, size, allocator);
    if (ret != RCUTILS_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG(rcutils_get_error_string().str);
        return rmw_convert_rcutils_ret_to_rmw_ret(ret);
    }
    return RMW_RET_OK;
}

} // namespace

extern "C" {

// Nodes ==========================================================================================================

rmw_ret_t rmw_get_node_names(const rmw_node_t* rmw_node,
                             rcutils_string_array_t* node_names,
                             rcutils_string_array_t* node_namespaces) {
    using ::rmw::iox2::Graph;
    using NodeImpl = ::rmw::iox2::Node;
    using ::rmw::iox2::unsafe_cast;

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(node_names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*node_names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(node_namespaces, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*node_namespaces, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    auto node_impl_result = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (!node_impl_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto names_result = Graph{*node_impl}.node_names();
    if (!names_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to list node names");
        return RMW_RET_ERROR;
    }
    const auto& names = names_result.value();

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto result = init_string_array(node_names, names.size(), &allocator);
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
        node_names->data[i] = rcutils_strdup(name.name.c_str(), allocator);
        if (!node_names->data[i]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to populate node name array");
            return RMW_RET_BAD_ALLOC;
        }

        node_namespaces->data[i] = rcutils_strdup(name.ns.c_str(), allocator);
        if (!node_namespaces->data[i]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to populate node namespace array");
            return RMW_RET_BAD_ALLOC;
        }

        ++i;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* rmw_node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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

rmw_ret_t rmw_count_publishers(const rmw_node_t* rmw_node, const char* topic_name, size_t* count) {
    using ::rmw::iox2::Graph;
    using NodeImpl = ::rmw::iox2::Node;
    using ::rmw::iox2::unsafe_cast;

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    auto node_impl_result = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (!node_impl_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto result = Graph{*node_impl}.count_publishers(topic_name);
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to count publishers");
        return RMW_RET_ERROR;
    }
    *count = result.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* rmw_node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace,
                                                    bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types) {
    using ::iox2::MessagingPattern;

    (void)no_demangle; // not used

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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

rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* rmw_node,
                                           rcutils_allocator_t* allocator,
                                           const char* topic_name,
                                           bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(publishers_info, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!rcutils_allocator_is_valid(allocator)) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if (rmw_topic_endpoint_info_array_check_zero(publishers_info) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::Graph;
    using NodeImpl = ::rmw::iox2::Node;
    using ::rmw::iox2::unsafe_cast;

    auto node_impl_result = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (!node_impl_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto result = Graph{*node_impl}.publishers_info(topic_name);
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get publishers info");
        return RMW_RET_ERROR;
    }

    return fill_endpoint_info_array(publishers_info, result.value(), RMW_ENDPOINT_PUBLISHER, allocator);
}

// Subscribers ======================================================================================================

rmw_ret_t rmw_count_subscribers(const rmw_node_t* rmw_node, const char* topic_name, size_t* count) {
    using ::rmw::iox2::Graph;
    using NodeImpl = ::rmw::iox2::Node;
    using ::rmw::iox2::unsafe_cast;

    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    auto node_impl_result = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (!node_impl_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto result = Graph{*node_impl}.count_subscribers(topic_name);
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to count subscribers");
        return RMW_RET_ERROR;
    }
    *count = result.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* rmw_node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace,
                                                     bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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

rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* rmw_node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name,
                                              bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(subscriptions_info, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!rcutils_allocator_is_valid(allocator)) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    if (rmw_topic_endpoint_info_array_check_zero(subscriptions_info) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::Graph;
    using NodeImpl = ::rmw::iox2::Node;
    using ::rmw::iox2::unsafe_cast;

    auto node_impl_result = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (!node_impl_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto result = Graph{*node_impl}.subscriptions_info(topic_name);
    if (!result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get subscriptions info");
        return RMW_RET_ERROR;
    }

    return fill_endpoint_info_array(subscriptions_info, result.value(), RMW_ENDPOINT_SUBSCRIPTION, allocator);
}

// Topics ===========================================================================================================

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* rmw_node,
                                        rcutils_allocator_t* allocator,
                                        bool no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types) {
    // Invariants -----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(topic_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (!rcutils_allocator_is_valid(allocator)) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    // A zero-initialized `rmw_names_and_types_t` has `types == NULL`, which is how
    // callers (e.g. `ros2 topic list`) pass it; validate that form rather than
    // dereferencing `types`.
    if (rmw_names_and_types_check_zero(topic_names_and_types) != RMW_RET_OK) {
        RMW_IOX2_CHAIN_ERROR_MSG("topic_names_and_types is not zero initialized");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::Graph;
    using NodeImpl = ::rmw::iox2::Node;
    using ::rmw::iox2::unsafe_cast;

    auto node_impl_result = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (!node_impl_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to get NodeImpl");
        return RMW_RET_ERROR;
    }
    auto& node_impl = node_impl_result.value();

    auto topics_result = Graph{*node_impl}.topic_names_and_types();
    if (!topics_result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to list topic names and types");
        return RMW_RET_ERROR;
    }
    const auto& topics = topics_result.value();

    auto init_result = rmw_names_and_types_init(topic_names_and_types, topics.size(), allocator);
    RMW_IOX2_ENSURE_OK(init_result);

    size_t index = 0;
    for (const auto& topic : topics) {
        // Allocate and copy topic name
        topic_names_and_types->names.data[index] = rcutils_strdup(topic.name.c_str(), *allocator);
        if (!topic_names_and_types->names.data[index]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
            return RMW_RET_BAD_ALLOC;
        }

        // Each topic carries exactly one type, stored in its own sub-array.
        if (rcutils_string_array_init(&topic_names_and_types->types[index], 1, allocator) != RCUTILS_RET_OK) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic types");
            return RMW_RET_BAD_ALLOC;
        }
        topic_names_and_types->types[index].data[0] = rcutils_strdup(topic.type.c_str(), *allocator);
        if (!topic_names_and_types->types[index].data[0]) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic type");
            return RMW_RET_BAD_ALLOC;
        }

        ++index;
    }

    return RMW_RET_OK;
}

// Services ==========================================================================================================

rmw_ret_t rmw_count_services(const rmw_node_t* rmw_node, const char* service_name, size_t* count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_SERVICE_NAME(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* rmw_node,
                                          rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(service_names_and_types, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(service_names_and_types->names, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_ZERO_STRING_ARRAY(*service_names_and_types->types, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* rmw_node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name,
                                                  const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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

rmw_ret_t rmw_count_clients(const rmw_node_t* rmw_node, const char* service_name, size_t* count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_VALID_SERVICE_NAME(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}


rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* rmw_node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
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

rmw_ret_t rmw_get_clients_info_by_service(const rmw_node_t* rmw_node,
                                          rcutils_allocator_t* allocator,
                                          const char* service_name,
                                          bool no_mangle,
                                          rmw_service_endpoint_info_array_t* clients_info) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(clients_info, RMW_RET_INVALID_ARGUMENT);
    if (rmw_service_endpoint_info_array_check_zero(clients_info) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_servers_info_by_service(const rmw_node_t* rmw_node,
                                          rcutils_allocator_t* allocator,
                                          const char* service_name,
                                          bool no_mangle,
                                          rmw_service_endpoint_info_array_t* servers_info) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_VALID_ALLOCATOR(allocator, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(service_name, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(servers_info, RMW_RET_INVALID_ARGUMENT);
    if (rmw_service_endpoint_info_array_check_zero(servers_info) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}
}
