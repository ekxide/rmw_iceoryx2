// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"
#include "iox/assertions_addendum.hpp"
#include "rmw/allocators.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"

extern "C" {

rmw_publisher_t* rmw_create_publisher(const rmw_node_t* rmw_node,
                                      const rosidl_message_type_support_t* type_support,
                                      const char* topic_name,
                                      const rmw_qos_profile_t* qos,
                                      const rmw_publisher_options_t* publisher_options) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node->context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node->context->impl, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, nullptr);
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
    using NodeImpl = ::rmw::iox2::Node;
    using PublisherImpl = ::rmw::iox2::Publisher;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Creating publisher to '%s'", topic_name);

    auto* rmw_publisher = rmw_publisher_allocate();
    if (rmw_publisher == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for rmw_publisher_t");
        return nullptr;
    }
    rmw_publisher->implementation_identifier = rmw_get_implementation_identifier();

    if (is_pod(type_support)) {
        rmw_publisher->can_loan_messages = true;
    } else {
        rmw_publisher->can_loan_messages = false;
        RMW_IOX2_LOG_DEBUG("Message type '%s' is not self-contained. Loaning disabled.",
                           type_support->get_type_description_func(type_support)->type_description.type_name.data);
    }

    if (auto ptr = allocate_copy(topic_name); ptr.has_error()) {
        rmw_publisher_free(rmw_publisher);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
        return nullptr;
    } else {
        rmw_publisher->topic_name = ptr.value();
    }

    auto node_impl = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (node_impl.has_error()) {
        rmw_publisher_free(rmw_publisher);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Node");
        return nullptr;
    }

    if (auto publisher_impl = allocate<PublisherImpl>(); publisher_impl.has_error()) {
        rmw_publisher_free(rmw_publisher);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for Publisher");
        return nullptr;
    } else {
        if (create_in_place<PublisherImpl>(publisher_impl.value(), *node_impl.value(), topic_name, type_support)
                .has_error()) {
            destruct<PublisherImpl>(publisher_impl.value());
            deallocate<PublisherImpl>(publisher_impl.value());
            rmw_publisher_free(rmw_publisher);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct Publisher");
            return nullptr;
        } else {
            rmw_publisher->data = publisher_impl.value();
        }
    }

    return rmw_publisher;
}

rmw_ret_t rmw_destroy_publisher(rmw_node_t* rmw_node, rmw_publisher_t* rmw_publisher) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using PublisherImpl = ::rmw::iox2::Publisher;

    RMW_IOX2_LOG_DEBUG("Destroying publisher to '%s'", rmw_publisher->topic_name);

    if (rmw_publisher->data) {
        destruct<PublisherImpl>(rmw_publisher->data);
        deallocate(rmw_publisher->data);
    }
    rmw_publisher_free(rmw_publisher);

    return RMW_RET_OK;
}

rmw_ret_t
rmw_publish(const rmw_publisher_t* rmw_publisher, const void* ros_message, rmw_publisher_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using PublisherImpl = ::rmw::iox2::Publisher;
    using ::rmw::iox2::serialized_message_size;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Publishing to '%s'", rmw_publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(rmw_publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }

    if (rmw_publisher->can_loan_messages) {
        // Self-contained. Copy message into payload.
        if (auto result =
                publisher_impl.value()->publish_copy(ros_message, publisher_impl.value()->unserialized_size());
            result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to publish copy");
            return RMW_RET_ERROR;
        }
    } else {
        // Non-self-contained. Serialize message into payload.
        auto type_support = publisher_impl.value()->typesupport();

        // The serialized size of THIS specific message
        auto serialized_size = serialized_message_size(ros_message, type_support);

        auto loan = publisher_impl.value()->loan(serialized_size);
        if (loan.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to loan bytes required for serialization");
            return RMW_RET_ERROR;
        }

        auto serialized_message = rmw_serialized_message_t{reinterpret_cast<uint8_t*>(loan.value()),
                                                           serialized_size,
                                                           serialized_size,
                                                           rcutils_get_default_allocator()};

        if (auto result = rmw_serialize(ros_message, type_support, &serialized_message); result != RMW_RET_OK) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to serialize into loaned payload");
            return RMW_RET_ERROR;
        }
        if (auto result = publisher_impl.value()->publish_loan(loan.value()); result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to publish serialized payload");
            return RMW_RET_ERROR;
        }
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_borrow_loaned_message(const rmw_publisher_t* rmw_publisher,
                                    const rosidl_message_type_support_t* type_support,
                                    void** ros_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher->data, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NULL(*ros_message, RMW_RET_INVALID_ARGUMENT);

    // TODO: ENSURE PUBLISHER CAN LOAN

    // Implementation -------------------------------------------------------------------------------
    using PublisherImpl = ::rmw::iox2::Publisher;
    using ::rmw::iox2::message_size;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Borrowing loan from '%s'", rmw_publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(rmw_publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }

    auto loan = publisher_impl.value()->loan(message_size(publisher_impl.value()->typesupport()));
    if (loan.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to loan memory for publisher payload");
        return RMW_RET_ERROR;
    }
    *ros_message = loan.value();

    return RMW_RET_OK;
}

rmw_ret_t rmw_return_loaned_message_from_publisher(const rmw_publisher_t* rmw_publisher, void* loaned_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // TODO: ENSURE PUBLISHER CAN LOAN

    // Implementation -------------------------------------------------------------------------------
    using PublisherImpl = ::rmw::iox2::Publisher;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Returning loan to '%s'", rmw_publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(rmw_publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }

    if (auto result = publisher_impl.value()->return_loan(loaned_message); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to return loaned message to publisher");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_publish_loaned_message(const rmw_publisher_t* rmw_publisher,
                                     void* ros_message,
                                     rmw_publisher_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using PublisherImpl = ::rmw::iox2::Publisher;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Publishing loan to '%s'", rmw_publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(rmw_publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }

    if (auto result = publisher_impl.value()->publish_loan(ros_message); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to publish loaned message");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* rmw_publisher,
                                         const rmw_serialized_message_t* serialized_message,
                                         rmw_publisher_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using PublisherImpl = ::rmw::iox2::Publisher;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Publishing pre-serialized message to '%s'", rmw_publisher->topic_name);

    auto publisher_impl = unsafe_cast<PublisherImpl*>(rmw_publisher->data);
    if (publisher_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }

    // Copy serialized payload into `iceoryx2` payload
    // WARNING: This publish variant is usable if ONLY serialized payloads are published on this topic.
    if (auto result =
            publisher_impl.value()->publish_copy(serialized_message->buffer, serialized_message->buffer_length);
        result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to publish copy");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t* rmw_publisher, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    *qos = rmw_qos_profile_default;

    return RMW_RET_OK;
}

rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* rmw_publisher, size_t* subscription_count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(subscription_count, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

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

rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t* rmw_publisher) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_wait_for_all_acked(const rmw_publisher_t* rmw_publisher, rmw_time_t wait_timeout) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_publisher_get_network_flow_endpoints(const rmw_publisher_t* rmw_publisher,
                                                   rcutils_allocator_t* allocator,
                                                   rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
    return RMW_RET_UNSUPPORTED;
}
}
