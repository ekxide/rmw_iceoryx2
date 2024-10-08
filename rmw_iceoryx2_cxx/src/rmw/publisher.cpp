// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"

extern "C" {

rmw_publisher_t* rmw_create_publisher(const rmw_node_t* node,
                                      const rosidl_message_type_support_t* type_support,
                                      const char* topic_name,
                                      const rmw_qos_profile_t* /*NOT SUPPORTED*/,
                                      const rmw_publisher_options_t* /*NOT SUPPORTED*/) {
    using rmw::iox2::allocate;
    using rmw::iox2::allocate_copy;
    using rmw::iox2::construct;
    using rmw::iox2::deallocate;
    using rmw::iox2::NodeImpl;
    using rmw::iox2::PublisherImpl;
    using rmw::iox2::unsafe_cast;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_create_publisher: node",
                                          node->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return nullptr);


    auto* publisher = rmw_publisher_allocate();
    if (publisher == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_publisher_");
        return nullptr;
    }

    publisher->implementation_identifier = rmw_get_implementation_identifier();
    publisher->can_loan_messages = true;

    if (auto ptr = allocate_copy(topic_name); ptr.has_error()) {
        rmw_publisher_free(publisher);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
        return nullptr;
    } else {
        publisher->topic_name = ptr.value();
    }

    auto node_impl = unsafe_cast<NodeImpl*>(node->data);
    if (node_impl.has_error()) {
        rmw_publisher_free(publisher);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve NodeImpl");
        return nullptr;
    }

    if (auto ptr = allocate<PublisherImpl>(); ptr.has_error()) {
        rmw_publisher_free(publisher);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for PublisherImpl");
        return nullptr;
    } else {
        if (construct<PublisherImpl>(ptr.value(), *node_impl.value(), topic_name, type_support->typesupport_identifier)
                .has_error()) {
            deallocate<PublisherImpl>(ptr.value());
            rmw_publisher_free(publisher);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct PublisherImpl");
            return nullptr;
        } else {
            publisher->data = ptr.value();
        }
    }

    return publisher;
}

rmw_ret_t rmw_destroy_publisher(rmw_node_t* node, rmw_publisher_t* publisher) {
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using rmw::iox2::PublisherImpl;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_destroy_publisher: publisher",
                                          publisher->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INVALID_ARGUMENT);

    if (publisher->data) {
        destruct<PublisherImpl>(publisher->data);
        deallocate(publisher->data);
    }
    rmw_publisher_free(publisher);

    return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher, size_t* subscription_count) {
    IOX_TODO();
}

rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t* publisher, rmw_qos_profile_t* qos) {
    IOX_TODO();
}

rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t* publisher,
                                    const rosidl_message_type_support_t* type_support,
                                    void** ros_message) {
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_borrow_loaned_message: publisher",
                                          publisher->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_ERROR);


    IOX_TODO();
}

rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t* publisher, void* loaned_message) {
    IOX_TODO();
}

rmw_ret_t
rmw_publish(const rmw_publisher_t* publisher, const void* ros_message, rmw_publisher_allocation_t* allocation) {
    IOX_TODO();
}

rmw_ret_t rmw_publish_loaned_message(const rmw_publisher_t* publisher,
                                     void* ros_message,
                                     rmw_publisher_allocation_t* allocation) {
    IOX_TODO();
}

rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                         const rmw_serialized_message_t* serialized_message,
                                         rmw_publisher_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_serialized_message_size(const rosidl_message_type_support_t* type_support,
                                          const rosidl_runtime_c__Sequence__bound* message_bounds,
                                          size_t* size) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_init_publisher_allocation(const rosidl_message_type_support_t* type_support,
                                        const rosidl_runtime_c__Sequence__bound* message_bounds,
                                        rmw_publisher_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t* publisher) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_wait_for_all_acked(const rmw_publisher_t* publisher, rmw_time_t wait_timeout) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_get_network_flow_endpoints(const rmw_publisher_t* publisher,
                                                   rcutils_allocator_t* allocator,
                                                   rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
    return RMW_RET_UNSUPPORTED;
}
}
