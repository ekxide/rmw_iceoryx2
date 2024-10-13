// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox/assertions_addendum.hpp"
#include "rmw/dynamic_message_type_support.h"
#include "rmw/get_network_flow_endpoints.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

extern "C" {

rmw_subscription_t* rmw_create_subscription(const rmw_node_t* node,
                                            const rosidl_message_type_support_t* type_support,
                                            const char* topic_name,
                                            const rmw_qos_profile_t* qos_policies,
                                            const rmw_subscription_options_t* subscription_options) {
    using rmw::iox2::allocate;
    using rmw::iox2::allocate_copy;
    using rmw::iox2::construct;
    using rmw::iox2::deallocate;
    using rmw::iox2::NodeImpl;
    using rmw::iox2::SubscriberImpl;
    using rmw::iox2::unsafe_cast;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node->context, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(node->context->impl, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(type_support, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_create_subscriber: node",
                                          node->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return nullptr);

    auto* subscription = rmw_subscription_allocate();
    if (subscription == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocator memoery for rmw_subscription_t");
        return nullptr;
    }

    subscription->implementation_identifier = rmw_get_implementation_identifier();
    subscription->can_loan_messages = true;

    if (auto ptr = allocate_copy(topic_name); ptr.has_error()) {
        rmw_subscription_free(subscription);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
        return nullptr;
    } else {
        subscription->topic_name = ptr.value();
    }

    auto node_impl = unsafe_cast<NodeImpl*>(node->data);
    if (node_impl.has_error()) {
        rmw_subscription_free(subscription);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve NodeImpl");
        return nullptr;
    }

    if (auto ptr = allocate<SubscriberImpl>(); ptr.has_error()) {
        rmw_subscription_free(subscription);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for SubscriberImpl");
        return nullptr;
    } else {
        if (construct<SubscriberImpl>(ptr.value(),
                                      *node_impl.value(),
                                      node->context->impl->id(),
                                      topic_name,
                                      type_support->typesupport_identifier)
                .has_error()) {
            deallocate<SubscriberImpl>(ptr.value());
            rmw_subscription_free(subscription);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct SubscriberImpl");
            return nullptr;
        } else {
            subscription->data = ptr.value();
        }
    }

    return subscription;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription) {
    using rmw::iox2::deallocate;
    using rmw::iox2::destruct;
    using rmw::iox2::SubscriberImpl;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_destroy_subscription: subscription",
                                          subscription->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INVALID_ARGUMENT);

    if (subscription->data) {
        destruct<SubscriberImpl>(subscription->data);
        deallocate(subscription->data);
    }
    rmw_subscription_free(subscription);

    return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription, size_t* publisher_count) {
    IOX_TODO();
}

rmw_ret_t rmw_subscription_get_actual_qos(const rmw_subscription_t* subscription, rmw_qos_profile_t* qos) {
    IOX_TODO();
}

rmw_ret_t rmw_take_loaned_message(const rmw_subscription_t* subscription,
                                  void** loaned_message,
                                  bool* taken,
                                  rmw_subscription_allocation_t* allocation) {
    IOX_TODO();
}

rmw_ret_t rmw_take_loaned_message_with_info(const rmw_subscription_t* subscription,
                                            void** loaned_message,
                                            bool* taken,
                                            rmw_message_info_t* message_info,
                                            rmw_subscription_allocation_t* allocation) {
    IOX_TODO();
}

rmw_ret_t rmw_return_loaned_message_from_subscription(const rmw_subscription_t* subscription, void* loaned_message) {
    IOX_TODO();
}

rmw_ret_t rmw_take(const rmw_subscription_t* subscription,
                   void* ros_message,
                   bool* taken,
                   rmw_subscription_allocation_t* allocation) {
    IOX_TODO();
}

rmw_ret_t rmw_take_with_info(const rmw_subscription_t* subscription,
                             void* ros_message,
                             bool* taken,
                             rmw_message_info_t* message_info,
                             rmw_subscription_allocation_t* allocation) {
    IOX_TODO();
}


rmw_ret_t rmw_take_sequence(const rmw_subscription_t* subscription,
                            size_t count,
                            rmw_message_sequence_t* message_sequence,
                            rmw_message_info_sequence_t* message_info_sequence,
                            size_t* taken,
                            rmw_subscription_allocation_t* allocation) {
    IOX_TODO();
}

rmw_ret_t rmw_take_serialized_message(const rmw_subscription_t* subscription,
                                      rmw_serialized_message_t* serialized_message,
                                      bool* taken,
                                      rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_serialized_message_with_info(const rmw_subscription_t* subscription,
                                                rmw_serialized_message_t* serialized_message,
                                                bool* taken,
                                                rmw_message_info_t* message_info,
                                                rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_dynamic_message(const rmw_subscription_t* subscription,
                                   rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message,
                                   bool* taken,
                                   rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_dynamic_message_with_info(const rmw_subscription_t* subscription,
                                             rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message,
                                             bool* taken,
                                             rmw_message_info_t* message_info,
                                             rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_init_subscription_allocation(const rosidl_message_type_support_t* type_support,
                                           const rosidl_runtime_c__Sequence__bound* message_bounds,
                                           rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_set_content_filter(rmw_subscription_t* subscription,
                                              const rmw_subscription_content_filter_options_t* options) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_content_filter(const rmw_subscription_t* subscription,
                                              rcutils_allocator_t* allocator,
                                              rmw_subscription_content_filter_options_t* options) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_set_on_new_message_callback(rmw_subscription_t* subscription,
                                                       rmw_event_callback_t callback,
                                                       const void* user_data) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_network_flow_endpoints(const rmw_subscription_t* subscription,
                                                      rcutils_allocator_t* allocator,
                                                      rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
    return RMW_RET_UNSUPPORTED;
}
}
