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
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/allocator_helpers.hpp"
#include "rmw_iceoryx2_cxx/create.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/waitset_impl.hpp"

extern "C" {
rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* context, size_t max_conditions) {
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::WaitSetImpl;

    (void)max_conditions; // not needed

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(context, nullptr);
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(context->impl, nullptr);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_create_wait_set: context",
                                          context->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return nullptr);

    rmw_wait_set_t* wait_set = rmw_wait_set_allocate();
    if (!wait_set) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for waitset handle");
        return nullptr;
    }
    wait_set->implementation_identifier = rmw_get_implementation_identifier();

    if (auto ptr = allocate<WaitSetImpl>(); ptr.has_error()) {
        rmw_wait_set_free(wait_set);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for WaitSetImpl");
        return nullptr;
    } else {
        if (create_in_place<WaitSetImpl>(ptr.value(), *context->impl).has_error()) {
            destruct<WaitSetImpl>(ptr.value());
            deallocate<WaitSetImpl>(ptr.value());
            rmw_wait_set_free(wait_set);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct WaitSetImpl");
            return nullptr;
        } else {
            wait_set->data = ptr.value();
        }
    }

    return wait_set;
}

rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* wait_set) {
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using ::rmw::iox2::WaitSetImpl;

    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_destroy_wait_set: context",
                                          wait_set->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    if (wait_set->data) {
        destruct<WaitSetImpl>(wait_set->data);
        deallocate(wait_set->data);
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_wait(rmw_subscriptions_t* subscriptions,
                   rmw_guard_conditions_t* guard_conditions,
                   rmw_services_t* /* NOT SUPPORTED */,
                   rmw_clients_t* /* NOT SUPPORTED */,
                   rmw_events_t* /* NOT SUPPORTED */,
                   rmw_wait_set_t* wait_set,
                   const rmw_time_t* wait_timeout) {
    using ::iox::units::Duration;
    using ::rmw::iox2::GuardConditionImpl;
    using ::rmw::iox2::SubscriberImpl;
    using ::rmw::iox2::unsafe_cast;
    using ::rmw::iox2::WaitSetImpl;

    // TODO: Null checks for waitables?
    RMW_IOX2_CHECK_ARGUMENT_FOR_NULL(wait_set, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_CHECK_TYPE_IDENTIFIERS_MATCH("rmw_wait: wait_set",
                                          wait_set->implementation_identifier,
                                          rmw_get_implementation_identifier(),
                                          return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    auto ptr = unsafe_cast<WaitSetImpl*>(wait_set->data);
    if (ptr.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve WaitSetImpl");
        return RMW_RET_ERROR;
    }
    auto waitset_impl = ptr.value();

    // Attach all guard_conditions to waitset
    if (guard_conditions) {
        for (size_t i = 0; i < guard_conditions->guard_condition_count; i++) {
            auto guard_condition = unsafe_cast<GuardConditionImpl*>(guard_conditions->guard_conditions[i]);
            if (guard_condition.has_error()) {
                // TODO: maybe detach previously attached elements? Detach all?
                RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve GuardConditionImpl");
                return RMW_RET_ERROR;
            }
            if (auto result = waitset_impl->attach(*guard_condition.value()); result.has_error()) {
                // TODO: maybe detach previously attached elements? Detach all?
                RMW_IOX2_CHAIN_ERROR_MSG("failed to attach GuardConditionImpl to WaitSetImpl");
                return RMW_RET_ERROR;
            }
        }
    }

    // Attach all subscriptions to waitset
    if (subscriptions) {
        for (size_t i = 0; i < subscriptions->subscriber_count; i++) {
            auto subscriber = unsafe_cast<SubscriberImpl*>(subscriptions->subscribers[i]);
            if (subscriber.has_error()) {
                // TODO: maybe detach previously attached elements? Detach all?
                RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve SubscriberImpl");
                return RMW_RET_ERROR;
            }
            if (auto result = waitset_impl->attach(*subscriber.value()); result.has_error()) {
                // TODO: maybe detach previously attached elements? Detach all?
                RMW_IOX2_CHAIN_ERROR_MSG("failed to attach SubscriberImpl to WaitSetImpl");
                return RMW_RET_ERROR;
            }
        }
    }

    // Wait and process
    auto secs = wait_timeout ? Duration::fromSeconds(wait_timeout->sec) : Duration::fromSeconds(0);
    auto nsecs = wait_timeout ? Duration::fromNanoseconds(wait_timeout->nsec) : Duration::fromNanoseconds(0);
    auto timeout = secs + nsecs;

    // TODO: indicate timeout
    if (auto result = waitset_impl->wait(timeout); result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("wait failure");
        return RMW_RET_ERROR;
    }

    return RMW_RET_OK;
}
}
