// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/allocators.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw_iceoryx2_cxx/allocator.hpp"
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/ensure.hpp"
#include "rmw_iceoryx2_cxx/error_message.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/publisher_impl.hpp"
#include "rmw_iceoryx2_cxx/log.hpp"
#include "rmw_iceoryx2_cxx/message/introspect.hpp"

extern "C" {

rmw_publisher_t* rmw_create_publisher(const rmw_node_t* node,
                                      const rosidl_message_type_support_t* type_support,
                                      const char* topic_name,
                                      const rmw_qos_profile_t* qos,
                                      const rmw_publisher_options_t* publisher_options) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(node->context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(node->context->impl, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, nullptr);
    RMW_IOX2_ENSURE_VALID_TYPESUPPORT(type_support, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(qos, nullptr);
    RMW_IOX2_ENSURE_VALID_QOS(qos, nullptr);
    if (!qos->avoid_ros_namespace_conventions) {
        RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, nullptr);
    }
    RMW_IOX2_ENSURE_NOT_NULL(publisher_options, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::allocate_copy;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::is_pod;
    using ::rmw::iox2::message_size;
    using ::rmw::iox2::NodeImpl;
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Creating publisher to '%s'", topic_name);

    auto* publisher = rmw_publisher_allocate();
    if (publisher == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_publisher_t");
        return nullptr;
    }
    publisher->implementation_identifier = rmw_get_implementation_identifier();

    if (is_pod(type_support)) {
        publisher->can_loan_messages = true;
    } else {
        publisher->can_loan_messages = false;
        RMW_IOX2_LOG_DEBUG("Message type '%s' is not self-contained. Loaning disabled.",
                           type_support->get_type_description_func(type_support)->type_description.type_name.data);
    }

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
        if (create_in_place<PublisherImpl>(ptr.value(),
                                           *node_impl.value(),
                                           topic_name,
                                           type_support->typesupport_identifier,
                                           message_size(type_support))
                .has_error()) {
            destruct<PublisherImpl>(ptr.value());
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
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::PublisherImpl;

    RMW_IOX2_LOG_DEBUG("Destroying publisher to '%s'", publisher->topic_name);

    if (publisher->data) {
        destruct<PublisherImpl>(publisher->data);
        deallocate(publisher->data);
    }
    rmw_publisher_free(publisher);

    return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher, size_t* subscription_count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t* publisher, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    *qos = rmw_qos_profile_default;

    return RMW_RET_OK;
}

rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t* publisher,
                                    const rosidl_message_type_support_t* type_support,
                                    void** ros_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(publisher->data, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NULL(*ros_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("New loan from '%s'", publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve PublisherImpl");
        return RMW_RET_ERROR;
    }

    auto loan = publisher_impl.value()->loan();
    if (loan.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to loan memory for publisher payload");
        return RMW_RET_ERROR;
    }
    *ros_message = loan.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t* publisher, void* loaned_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Releasing loan to '%s'", publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve PublisherImpl");
        return RMW_RET_ERROR;
    }

    if (auto result = publisher_impl.value()->return_loan(loaned_message); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to return loaned message to publisher");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t
rmw_publish(const rmw_publisher_t* publisher, const void* ros_message, rmw_publisher_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Publishing copy to '%s'", publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve PublisherImpl");
        return RMW_RET_ERROR;
    }

    if (publisher->can_loan_messages) {
        // Publishers with loanable message types can be simply copied into shared-memory.
        if (auto result = publisher_impl.value()->publish_copy(ros_message, publisher_impl.value()->payload_size());
            result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to publish copy");
            return RMW_RET_ERROR;
        }
    } else {
        // Non-loanable message types must be serialized
        RMW_IOX2_LOG_WARN("skipping publish to topic '%s'", publisher->topic_name);
        RMW_IOX2_LOG_WARN("non-self-contained message types are not yet supported");
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_publish_loaned_message(const rmw_publisher_t* publisher,
                                     void* ros_message,
                                     rmw_publisher_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::PublisherImpl;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Publishing loan to '%s'", publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve PublisherImpl");
        return RMW_RET_ERROR;
    }

    if (auto result = publisher_impl.value()->publish_loan(ros_message); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to publish loaned message");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                         const rmw_serialized_message_t* serialized_message,
                                         rmw_publisher_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
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
