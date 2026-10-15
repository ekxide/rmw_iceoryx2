// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw/event.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/events.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/subscriber.hpp"

namespace
{

void init_event(rmw_event_t* rmw_event, rmw_event_type_t event_type, ::rmw::iox2::Events* events) {
    rmw_event->implementation_identifier = rmw_get_implementation_identifier();
    rmw_event->data = events;
    rmw_event->event_type = event_type;
}

} // namespace

extern "C" {

rmw_ret_t
rmw_publisher_event_init(rmw_event_t* rmw_event, const rmw_publisher_t* rmw_publisher, rmw_event_type_t event_type) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_publisher, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_publisher->implementation_identifier, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::unsafe_cast;
    using PublisherImpl = ::rmw::iox2::Publisher;

    if (!rmw_event_type_is_supported(event_type)) {
        return RMW_RET_UNSUPPORTED;
    }
    auto publisher_impl = unsafe_cast<PublisherImpl*>(rmw_publisher->data);
    if (!publisher_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Publisher");
        return RMW_RET_ERROR;
    }
    auto events = publisher_impl.value()->events();
    if (!events.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the events of the Publisher");
        return RMW_RET_ERROR;
    }
    init_event(rmw_event, event_type, events.value());
    return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_event_init(rmw_event_t* rmw_event,
                                      const rmw_subscription_t* rmw_subscription,
                                      rmw_event_type_t event_type) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_subscription, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_subscription->implementation_identifier, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::unsafe_cast;
    using SubscriberImpl = ::rmw::iox2::Subscriber;

    if (!rmw_event_type_is_supported(event_type)) {
        return RMW_RET_UNSUPPORTED;
    }
    auto subscriber_impl = unsafe_cast<SubscriberImpl*>(rmw_subscription->data);
    if (!subscriber_impl.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Subscriber");
        return RMW_RET_ERROR;
    }
    auto events = subscriber_impl.value()->events();
    if (!events.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the events of the Subscriber");
        return RMW_RET_ERROR;
    }
    init_event(rmw_event, event_type, events.value());
    return RMW_RET_OK;
}

rmw_ret_t rmw_take_event(const rmw_event_t* rmw_event, void* event_info, bool* taken) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_event->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    RMW_IOX2_ENSURE_NOT_NULL(event_info, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_NOT_NULL(taken, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::Events;
    using ::rmw::iox2::unsafe_cast;

    auto events = unsafe_cast<Events*>(rmw_event->data);
    if (!events.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve the events");
        return RMW_RET_ERROR;
    }
    *taken = events.value()->take(rmw_event->event_type, event_info);
    return RMW_RET_OK;
}

rmw_ret_t rmw_event_fini(rmw_event_t* rmw_event) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_event, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_event->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    *rmw_event = rmw_get_zero_initialized_event();
    return RMW_RET_OK;
}

rmw_ret_t rmw_event_set_callback(rmw_event_t* rmw_event, rmw_event_callback_t callback, const void* user_data) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(user_data, RMW_RET_INVALID_ARGUMENT);

    // Implementation -------------------------------------------------------------------------------
    return RMW_RET_UNSUPPORTED;
}

bool rmw_event_type_is_supported(rmw_event_type_t rmw_event_type) {
    // Implementation -------------------------------------------------------------------------------
    return rmw_event_type == RMW_EVENT_PUBLICATION_MATCHED || rmw_event_type == RMW_EVENT_SUBSCRIPTION_MATCHED;
}
}
