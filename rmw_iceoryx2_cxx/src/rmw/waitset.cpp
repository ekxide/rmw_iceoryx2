// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/waitset.hpp"
#include "rmw/allocators.h"
#include "rmw/ret_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/allocator.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/ensure.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"

#include <set>

extern "C" {
rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* rmw_context, size_t max_conditions) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context, nullptr);
    RMW_IOX2_ENSURE_NOT_NULL(rmw_context->impl, nullptr);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_context->implementation_identifier, nullptr);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::allocate;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using WaitSetImpl = ::rmw::iox2::WaitSet;

    (void)max_conditions; // not needed

    RMW_IOX2_LOG_DEBUG("Creating waitset");

    rmw_wait_set_t* rmw_wait_set = rmw_wait_set_allocate();
    if (!rmw_wait_set) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for waitset handle");
        return nullptr;
    }
    rmw_wait_set->implementation_identifier = rmw_get_implementation_identifier();

    if (auto waitset_impl = allocate<WaitSetImpl>(); waitset_impl.has_error()) {
        rmw_wait_set_free(rmw_wait_set);
        RMW_IOX2_CHAIN_ERROR_MSG("failed to allocate memory for WaitSet");
        return nullptr;
    } else {
        if (create_in_place<WaitSetImpl>(waitset_impl.value(), *rmw_context->impl).has_error()) {
            destruct<WaitSetImpl>(waitset_impl.value());
            deallocate<WaitSetImpl>(waitset_impl.value());
            rmw_wait_set_free(rmw_wait_set);
            RMW_IOX2_CHAIN_ERROR_MSG("failed to construct WaitSet");
            return nullptr;
        } else {
            rmw_wait_set->data = waitset_impl.value();
        }
    }

    return rmw_wait_set;
}

rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* rmw_wait_set) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_wait_set, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_wait_set->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

    // Implementation -------------------------------------------------------------------------------
    using ::rmw::iox2::deallocate;
    using ::rmw::iox2::destruct;
    using WaitSetImpl = ::rmw::iox2::WaitSet;

    RMW_IOX2_LOG_DEBUG("Destroying waitset");

    if (rmw_wait_set->data) {
        destruct<WaitSetImpl>(rmw_wait_set->data);
        deallocate(rmw_wait_set->data);
    }

    return RMW_RET_OK;
}

rmw_ret_t rmw_wait(rmw_subscriptions_t* rmw_subscriptions,
                   rmw_guard_conditions_t* rmw_guard_conditions,
                   rmw_services_t* rmw_services,
                   rmw_clients_t* rmw_clients,
                   rmw_events_t* rmw_events,
                   rmw_wait_set_t* rmw_wait_set,
                   const rmw_time_t* wait_timeout) {
    // Invariants ----------------------------------------------------------------------------------
    RMW_IOX2_ENSURE_NOT_NULL(rmw_wait_set, RMW_RET_INVALID_ARGUMENT);
    RMW_IOX2_ENSURE_IMPLEMENTATION(rmw_wait_set->implementation_identifier, RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
    if (rmw_guard_conditions) {
        for (size_t index = 0; index < rmw_guard_conditions->guard_condition_count; index++) {
            if (rmw_guard_conditions->guard_conditions[index] == nullptr) {
                RMW_IOX2_CHAIN_ERROR_MSG("waitset input guard condition contains nullptr");
                return RMW_RET_INVALID_ARGUMENT;
            }
        }
    }
    if (rmw_subscriptions) {
        for (size_t index = 0; index < rmw_subscriptions->subscriber_count; index++) {
            if (rmw_subscriptions->subscribers[index] == nullptr) {
                RMW_IOX2_CHAIN_ERROR_MSG("waitset input subscriber contains nullptr");
                return RMW_RET_INVALID_ARGUMENT;
            }
        }
    }

    // Implementation -------------------------------------------------------------------------------
    using ::iox::units::Duration;
    using GuardConditionImpl = ::rmw::iox2::GuardCondition;
    using SubscriberImpl = ::rmw::iox2::Subscriber;
    using ::rmw::iox2::unsafe_cast;
    using ::rmw::iox2::WaitableEntity;
    using WaitSetImpl = ::rmw::iox2::WaitSet;

    iox::optional<Duration> timeout;
    if (wait_timeout) {
        timeout.emplace(Duration::fromSeconds(wait_timeout->sec) + Duration::fromNanoseconds(wait_timeout->nsec));
    }

    auto ptr = unsafe_cast<WaitSetImpl*>(rmw_wait_set->data);
    if (ptr.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve WaitSet");
        return RMW_RET_ERROR;
    }
    auto waitset_impl = ptr.value();

    // Attach all guard_conditions to waitset
    if (rmw_guard_conditions) {
        for (size_t index = 0; index < rmw_guard_conditions->guard_condition_count; index++) {
            auto guard_condition = unsafe_cast<GuardConditionImpl*>(rmw_guard_conditions->guard_conditions[index]);
            if (guard_condition.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve GuardCondition");
                return RMW_RET_ERROR;
            }
            if (auto result = waitset_impl->map(index, *guard_condition.value()); result.has_error()) {
                // TODO: maybe detach previously attached elements? Detach all?
                RMW_IOX2_CHAIN_ERROR_MSG("failed to attach GuardCondition to WaitSet");
                return RMW_RET_ERROR;
            }
        }
    }

    // Attach all subscriptions to waitset
    if (rmw_subscriptions) {
        for (size_t index = 0; index < rmw_subscriptions->subscriber_count; index++) {
            auto subscriber = unsafe_cast<SubscriberImpl*>(rmw_subscriptions->subscribers[index]);
            if (subscriber.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve Subscriber");
                return RMW_RET_ERROR;
            }
            if (auto result = waitset_impl->map(index, *subscriber.value()); result.has_error()) {
                // TODO: maybe detach previously attached elements? Detach all?
                RMW_IOX2_CHAIN_ERROR_MSG("failed to attach Subscriber to WaitSet");
                return RMW_RET_ERROR;
            }
        }
    }

    // Wait and process
    if (timeout.has_value()) {
        RMW_IOX2_LOG_DEBUG("Waiting on waitset (timeout=%lu)", timeout->toNanoseconds());
    } else {
        RMW_IOX2_LOG_DEBUG("Waiting on waitset (no timeout)");
    }
    auto wait_result = waitset_impl->wait(timeout);
    if (wait_result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG("waiting on waitset failed");
        return RMW_RET_ERROR;
    }

    // Reset all mappings - each wait call provides a different set of mappings
    waitset_impl->unmap_all();

    // Process triggers
    auto return_code = RMW_RET_TIMEOUT;
    auto triggers = std::move(wait_result.value());
    if (!triggers.empty()) {
        return_code = RMW_RET_OK;

        // Collect all triggered indices
        std::set<size_t> triggered_subscribers;
        std::set<size_t> triggered_guard_conditions;
        for (const auto& trigger : triggers) {
            switch (trigger.waitable_type) {
            case WaitableEntity::SUBSCRIBER:
                triggered_subscribers.insert(trigger.rmw_index);
                break;
            case WaitableEntity::GUARD_CONDITION:
                triggered_guard_conditions.insert(trigger.rmw_index);
                break;
            }
        }

        // Set non-triggered indices to nullptr
        if (rmw_subscriptions) {
            for (size_t index = 0; index < rmw_subscriptions->subscriber_count; index++) {
                if (triggered_subscribers.find(index) == triggered_subscribers.end()) {
                    rmw_subscriptions->subscribers[index] = nullptr;
                }
            }
        }
        if (rmw_guard_conditions) {
            for (size_t index = 0; index < rmw_guard_conditions->guard_condition_count; index++) {
                if (triggered_guard_conditions.find(index) == triggered_guard_conditions.end()) {
                    rmw_guard_conditions->guard_conditions[index] = nullptr;
                }
            }
        }
    }

    // Set all events to null (not supported yet)
    if (rmw_events) {
        for (size_t index = 0; index < rmw_events->event_count; index++) {
            rmw_events->events[index] = nullptr;
        }
    }
    if (rmw_services) {
        for (size_t index = 0; index < rmw_services->service_count; index++) {
            rmw_services->services[index] = nullptr;
        }
    }
    if (rmw_clients) {
        for (size_t index = 0; index < rmw_clients->client_count; index++) {
            rmw_clients->clients[index] = nullptr;
        }
    }

    return return_code;
}
}
