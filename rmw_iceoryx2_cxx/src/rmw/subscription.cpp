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
#include "rmw/dynamic_message_type_support.h"
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
#include "rmw_iceoryx2_cxx/impl/runtime/subscriber.hpp"

extern "C" {

rmw_subscription_t* rmw_create_subscription(const rmw_node_t* rmw_node,
                                            const rosidl_message_type_support_t* type_support,
                                            const char* topic_name,
                                            const rmw_qos_profile_t* qos_profile,
                                            const rmw_subscription_options_t* subscription_options) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node->context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node->context->impl, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(type_support, nullptr);
    RMW_IOX2_ENSURE_VALID_TYPESUPPORT(type_support, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(topic_name, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(qos_profile, nullptr);
    RMW_IOX2_ENSURE_VALID_QOS(qos_profile, nullptr);
    if (!qos_profile->avoid_ros_namespace_conventions) {
        RMW_IOX2_ENSURE_VALID_TOPIC_NAME(topic_name, nullptr);
    }
    RMW_IOX2_ENSURE_NOT_NULL(subscription_options, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::allocate_copy;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::is_pod;
    using NodeImpl = ::rmw::iox2::Node;
    using SubscriberImpl = ::rmw::iox2::Subscriber;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Creating subscription to '%s'", topic_name);

    auto* rmw_subscription = rmw_subscription_allocate();
    if (rmw_subscription == nullptr) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocator memoery for rmw_subscription_t");
        return nullptr;
    }
    rmw_subscription->implementation_identifier = rmw_get_implementation_identifier();

    if (is_pod(type_support)) {
        rmw_subscription->can_loan_messages = true;
    } else {
        rmw_subscription->can_loan_messages = false;
        RMW_IOX2_LOG_DEBUG("Message type '%s' is not self-contained. Loaning disabled.",
                           type_support->get_type_description_func(type_support)->type_description.type_name.data);
    }

    if (auto ptr = allocate_copy(topic_name); ptr.has_error()) {
        rmw_subscription_free(rmw_subscription);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for topic name");
        return nullptr;
    } else {
        rmw_subscription->topic_name = ptr.value();
    }

    auto node_impl = unsafe_cast<NodeImpl*>(rmw_node->data);
    if (node_impl.has_error()) {
        rmw_subscription_free(rmw_subscription);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Node");
        return nullptr;
    }

    if (auto subscriber_impl = allocate<SubscriberImpl>(); subscriber_impl.has_error()) {
        rmw_subscription_free(rmw_subscription);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for Subscriber");
        return nullptr;
    } else {
        if (create_in_place<SubscriberImpl>(subscriber_impl.value(), *node_impl.value(), topic_name, type_support)
                .has_error()) {
            destruct<SubscriberImpl>(subscriber_impl.value());
            deallocate<SubscriberImpl>(subscriber_impl.value());
            rmw_subscription_free(rmw_subscription);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct Subscriber");
            return nullptr;
        } else {
            rmw_subscription->data = subscriber_impl.value();
        }
    }

    return rmw_subscription;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t* rmw_node, rmw_subscription_t* rmw_subscription) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_node, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_node->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using SubscriberImpl = ::rmw::iox2::Subscriber;

    RMW_IOX2_LOG_DEBUG("Destroying subscription to '%s'", rmw_subscription->topic_name);

    if (rmw_subscription->data) {
        destruct<SubscriberImpl>(rmw_subscription->data);
        deallocate(rmw_subscription->data);
    }
    rmw_subscription_free(rmw_subscription);

    return RMW_RET_OK;
}

rmw_ret_t rmw_take(const rmw_subscription_t* rmw_subscription,
                   void* ros_message,
                   bool* taken,
                   rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using SubscriberImpl = ::rmw::iox2::Subscriber;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Taking from '%s'", rmw_subscription->topic_name);

    if (auto result = unsafe_cast<SubscriberImpl*>(rmw_subscription->data); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Subscriber");
        return RMW_RET_ERROR;
    } else {
        auto subscriber_impl = result.value();

        if (rmw_subscription->can_loan_messages) {
            // Self-contained. Copy payload into message.
            auto take_result = subscriber_impl->take_copy(ros_message);
            if (take_result.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG("failed to take copy from subscriber");
                return RMW_RET_ERROR;
            }
            *taken = take_result.value();
        } else {
            // Non-self-contained. Deserialize payload into message
            auto loan_result = subscriber_impl->take_loan();
            if (loan_result.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG("failed to take loan from subscriber");
                return RMW_RET_ERROR;
            } else {
                auto sample = std::move(loan_result.value());
                *taken = sample.has_value();

                if (sample.has_value()) {
                    auto typesupport = subscriber_impl->typesupport();
                    auto loan = std::move(sample.value());

                    auto serialized_message = rmw_serialized_message_t{
                        loan.bytes, loan.number_of_bytes, loan.number_of_bytes, rcutils_get_default_allocator()};

                    if (auto result = rmw_deserialize(&serialized_message, typesupport, ros_message);
                        result != RMW_RET_OK) {
                        RMW_IOX2_CHAIN_ERROR_MSG("failed to deserialize received message");
                        return RMW_RET_ERROR;
                    }

                    if (auto result = subscriber_impl->return_loan(loan.bytes); result.has_error()) {
                        RMW_IOX2_CHAIN_ERROR_MSG("failed to return loaned serialized payload");
                        return RMW_RET_ERROR;
                    }
                }
            }
        }
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_take_with_info(const rmw_subscription_t* rmw_subscription,
                             void* ros_message,
                             bool* taken,
                             rmw_message_info_t* message_info,
                             rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    return rmw_take(rmw_subscription, ros_message, taken, allocation);
}

rmw_ret_t rmw_take_loaned_message(const rmw_subscription_t* rmw_subscription,
                                  void** loaned_message,
                                  bool* taken,
                                  rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_CAN_LOAN(rmw_subscription, RMW_RET_UNSUPPORTED);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NULL(*loaned_message, RMW_RET_INVALID_ARGUMENT);

    if (!rmw_subscription->can_loan_messages) {
        RMW_IOX2_CHAIN_ERROR_MSG("non-self-contained messages do not support loaning");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    using SubscriberImpl = ::rmw::iox2::Subscriber;
    using ::rmw::iox2::unsafe_cast;
    (void)allocation; // not used

    if (!rmw_subscription->can_loan_messages) {
        RMW_IOX2_CHAIN_ERROR_MSG("attempted to take loan from subscription that does not support loaning");
        return RMW_RET_UNSUPPORTED;
    }

    RMW_IOX2_LOG_DEBUG("Taking loan from from '%s'", rmw_subscription->topic_name);

    auto subscriber_impl = unsafe_cast<SubscriberImpl*>(rmw_subscription->data);
    if (subscriber_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Subscriber");
        return RMW_RET_ERROR;
    }

    auto loan = subscriber_impl.value()->take_loan();
    if (loan.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to take sample from subscriber");
        return RMW_RET_ERROR;
    }

    auto payload = std::move(loan.value());
    if (payload.has_value()) {
        *loaned_message = static_cast<void*>(
            const_cast<uint8_t*>(static_cast<const uint8_t*>(payload->bytes))); // const cast forced by RMW API
        *taken = true;
    } else {
        *taken = false;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_take_loaned_message_with_info(const rmw_subscription_t* rmw_subscription,
                                            void** loaned_message,
                                            bool* taken,
                                            rmw_message_info_t* message_info,
                                            rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_CAN_LOAN(rmw_subscription, RMW_RET_UNSUPPORTED);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    (void)message_info; // TODO: support this

    return rmw_take_loaned_message(rmw_subscription, loaned_message, taken, allocation);
}

rmw_ret_t rmw_return_loaned_message_from_subscription(const rmw_subscription_t* rmw_subscription,
                                                      void* loaned_message) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_CAN_LOAN(rmw_subscription, RMW_RET_UNSUPPORTED);
    RMW_IOX2_ENSURE_NOT_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);

    if (!rmw_subscription->can_loan_messages) {
        RMW_IOX2_CHAIN_ERROR_MSG("non-self-contained messages do not support loaning");
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    using SubscriberImpl = ::rmw::iox2::Subscriber;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Releasing loan to '%s'", rmw_subscription->topic_name);

    auto subscriber_impl = unsafe_cast<SubscriberImpl*>(rmw_subscription->data);
    if (subscriber_impl.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Subscriber");
        return RMW_RET_ERROR;
    }

    if (auto result = subscriber_impl.value()->return_loan(loaned_message); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to return loaned message to publisher");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_take_serialized_message(const rmw_subscription_t* rmw_subscription,
                                      rmw_serialized_message_t* serialized_message,
                                      bool* taken,
                                      rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using SubscriberImpl = ::rmw::iox2::Subscriber;
    using ::rmw::iox2::unsafe_cast;

    RMW_IOX2_LOG_DEBUG("Taking serialized message from '%s'", rmw_subscription->topic_name);

    // Copy serialized payload into serialized message.
    // WARNING: This take variant is usable if ONLY serialized payloads are published on this topic.
    if (auto result = unsafe_cast<SubscriberImpl*>(rmw_subscription->data); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Subscriber");
        return RMW_RET_ERROR;
    } else {
        auto subscriber_impl = result.value();

        auto loan_result = subscriber_impl->take_loan();
        if (loan_result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to take loan from subscriber");
            return RMW_RET_ERROR;
        } else {
            auto sample = std::move(loan_result.value());
            *taken = sample.has_value();

            if (sample.has_value()) {
                auto loan = std::move(sample.value());

                if (auto result = rmw_serialized_message_resize(serialized_message, loan.number_of_bytes);
                    result != RMW_RET_OK) {
                    RMW_IOX2_CHAIN_ERROR_MSG("failed to resize serialized message to store received payload");
                    return RMW_RET_ERROR;
                }

                memcpy(serialized_message->buffer, loan.bytes, loan.number_of_bytes);

                if (auto result = subscriber_impl->return_loan(loan.bytes); result.has_error()) {
                    RMW_IOX2_CHAIN_ERROR_MSG("failed to return loaned serialized payload");
                    return RMW_RET_ERROR;
                }
            }
        }
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_take_sequence(const rmw_subscription_t* rmw_subscription,
                            size_t count,
                            rmw_message_sequence_t* message_sequence,
                            rmw_message_info_sequence_t* message_info_sequence,
                            size_t* taken,
                            rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(message_sequence, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(message_info_sequence, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);
    if (count == 0 || message_sequence->capacity < count || message_info_sequence->capacity < count) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_serialized_message_with_info(const rmw_subscription_t* rmw_subscription,
                                                rmw_serialized_message_t* serialized_message,
                                                bool* taken,
                                                rmw_message_info_t* message_info,
                                                rmw_subscription_allocation_t* allocation) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(serialized_message, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(message_info, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* rmw_subscription,
                                                    size_t* publisher_count) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(publisher_count, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_actual_qos(const rmw_subscription_t* rmw_subscription, rmw_qos_profile_t* qos) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(qos, RMW_RET_INVALID_ARGUMENT);

    // ementation -------------------------------------------------------------------------------
    *qos = rmw_qos_profile_default;

    return RMW_RET_OK;
}

rmw_ret_t rmw_take_dynamic_message(const rmw_subscription_t* rmw_subscription,
                                   rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message,
                                   bool* taken,
                                   rmw_subscription_allocation_t* allocation) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_dynamic_message_with_info(const rmw_subscription_t* rmw_subscription,
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

rmw_ret_t rmw_subscription_set_content_filter(rmw_subscription_t* rmw_subscription,
                                              const rmw_subscription_content_filter_options_t* options) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_content_filter(const rmw_subscription_t* rmw_subscription,
                                              rcutils_allocator_t* allocator,
                                              rmw_subscription_content_filter_options_t* options) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_set_on_new_message_callback(rmw_subscription_t* rmw_subscription,
                                                       rmw_event_callback_t callback,
                                                       const void* user_data) {
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_get_network_flow_endpoints(const rmw_subscription_t* rmw_subscription,
                                                      rcutils_allocator_t* allocator,
                                                      rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
    return RMW_RET_UNSUPPORTED;
}
}
