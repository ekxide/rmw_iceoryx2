// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/waitset_impl.hpp"

#include "iox2/callback_progression.hpp"
#include "iox2/waitset.hpp"
#include "rcutils/logging_macros.h"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

namespace rmw::iox2
{

WaitSetImpl::WaitSetImpl(CreationLock, iox::optional<WaitSetError>& error, ContextImpl& context)
    : m_context{context} {
    auto waitset = Iceoryx2::WaitSet::create();
    if (waitset.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(waitset.error()));
        error.emplace(ErrorType::WAITSET_CREATION_FAILURE);
        return;
    }
    m_waitset.emplace(std::move(waitset.value()));
}

auto WaitSetImpl::map(RmwIndex rmw_index, GuardConditionImpl& guard_condition) -> iox::expected<void, WaitSetError> {
    using ::iox::err;
    using ::iox::ok;

    if (auto result = get_storage_index<GuardConditionListener>(guard_condition.service_name()); result.has_error()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        map_stored_listener(WaitableEntity::GUARD_CONDITION, storage_index, rmw_index);
        return ok();
    }
}

auto WaitSetImpl::map(RmwIndex rmw_index, SubscriberImpl& subscriber) -> iox::expected<void, WaitSetError> {
    using ::iox::err;
    using ::iox::ok;

    if (auto result = get_storage_index<SubscriberListener>(subscriber.service_name()); result.has_error()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        map_stored_listener(WaitableEntity::SUBSCRIBER, storage_index, rmw_index);
        return ok();
    }
}

auto WaitSetImpl::map_stored_listener(WaitableEntity waitable_type,
                                      StorageIndex storage_index,
                                      RmwIndex rmw_index) -> void {
    auto it = std::find_if(
        m_mapped_listeners.begin(), m_mapped_listeners.end(), [waitable_type, storage_index](const auto& staged) {
            return staged.waitable_type == waitable_type && staged.storage_index == storage_index;
        });

    if (it == m_mapped_listeners.end()) {
        m_mapped_listeners.push_back(RmwMapping{waitable_type, storage_index, rmw_index});
    }
}

auto WaitSetImpl::unmap_all() -> void {
    // Detaching removes the mapping, but the listener remains in the storage for re-use in subsequent calls.
    m_mapped_listeners.clear();
}


auto WaitSetImpl::wait(const iox::optional<Duration>& timeout)
    -> iox::expected<iox::optional<TriggeredWaitable>, ErrorType> {
    using ::iox::err;
    using ::iox::ok;
    using ::iox2::CallbackProgression;

    // Context for this specific wait call.
    // Cleaned up automatically at end of scope, detaching all attachments from the waitset.
    WaitContext ctx;

    // Attach the timeout to the waitset
    if (non_zero_timeout(timeout)) {
        if (auto result = attach_timeout(timeout.value(), ctx); result.has_error()) {
            return err(result.error());
        }
    }

    // Attached all previously mapped listeners
    if (auto result = attach_mapped_listeners(ctx); result.has_error()) {
        return err(result.error());
    }

    // Callback to process events received on listeners attached to waitset
    auto on_event = [this, &ctx](auto id) -> CallbackProgression {
        // Check for timeout
        if (ctx.attached_timeout.has_value() && ctx.attached_timeout->id() == id) {
            return CallbackProgression::Stop;
        }

        // Find the triggered attachment
        for (const auto& attachment : ctx.attached_listeners) {
            if (attachment.id() == id) {
                // This waitable was triggered. Drain all events. The number of triggers is irrelevant.
                if (auto result =
                        process_trigger(attachment.mapping().waitable_type, attachment.mapping().storage_index);
                    result.has_error()) {
                    RCUTILS_LOG_ERROR_NAMED("rmw_iceoryx2", "Failed to process trigger from a waitset attachment");
                    return CallbackProgression::Stop;
                } else {
                    if (!ctx.result.has_value()) {
                        ctx.result.emplace(TriggeredWaitable{attachment.mapping()});
                    }
                    return CallbackProgression::Stop;
                }
            }
        }

        // If this occurs there is a bug in WaitsetImpl.
        // Continue looking for notifications from other attachments so as not to hinder functionality.
        RCUTILS_LOG_ERROR_NAMED("rmw_iceoryx2", "Waitset was triggered by an unmapped subscriber or guard condition");
        return CallbackProgression::Continue;
    };

    // If timeout is non-zero, block and wait, otherwise check for events and return immediately.
    if (auto result =
            should_block(timeout) ? m_waitset->wait_and_process(on_event) : m_waitset->wait_and_process_once(on_event);
        result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::WAIT_FAILURE);
    }

    return ok(ctx.result);
}

auto WaitSetImpl::non_zero_timeout(const iox::optional<Duration>& timeout) const -> bool {
    return timeout.has_value() && timeout.value() != Duration::zero();
}

auto WaitSetImpl::should_block(const iox::optional<Duration>& timeout) const -> bool {
    return !(timeout.has_value() && timeout.value() == Duration::zero());
}

auto WaitSetImpl::attach_timeout(const Duration& timeout, WaitContext& ctx) -> ::iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    auto guard = m_waitset->attach_interval(timeout);
    if (guard.has_error()) {
        return err(ErrorType::ATTACHMENT_FAILURE);
    }
    ctx.attached_timeout.emplace(std::move(guard.value()));
    return ok();
}

auto WaitSetImpl::attach_mapped_listeners(WaitContext& ctx) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    for (const auto& staged : m_mapped_listeners) {
        auto result = attach_mapped_listener(staged);
        if (result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to attach mapped listeners to waitset");
            return err(result.error());
        }
        ctx.attached_listeners.push_back(std::move(result.value()));
    }
    return ok();
}

auto WaitSetImpl::attach_mapped_listener(const RmwMapping& mapping) -> iox::expected<AttachmentDetails, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    switch (mapping.waitable_type) {
    case WaitableEntity::GUARD_CONDITION:
        return attach_mapped_listener_impl<GuardConditionListener>(mapping);
    case WaitableEntity::SUBSCRIBER:
        return attach_mapped_listener_impl<SubscriberListener>(mapping);
    default:
        RMW_IOX2_CHAIN_ERROR_MSG("attempted to attach an unknown waitable type");
        return err(ErrorType::INVALID_WAITABLE_TYPE);
    }
}

auto WaitSetImpl::process_trigger(const WaitableEntity waitable_type,
                                  const StorageIndex storage_index) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    // Drain all events from the trigger.
    // The value nor number of triggers is irrelevant, so no callback logic required.
    auto drain_events = [](auto& listener) -> iox::expected<void, ErrorType> {
        if (auto result = listener.try_wait_all([&](auto) {}); result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve events from listener");
            return err(ErrorType::LISTENER_FAILURE);
        }
        return ok();
    };

    // Retrieve the listener from the corresponding storage and drain all of its events
    switch (waitable_type) {
    case WaitableEntity::GUARD_CONDITION: {
        if (auto result = get_stored_listener<GuardConditionListener>(storage_index); result.has_value()) {
            auto& listener_details = result.value();
            if (auto result = drain_events(listener_details->listener); result.has_error()) {
                return err(result.error());
            }
        } else {
            RMW_IOX2_CHAIN_ERROR_MSG("unable to find guard condition listener at provided index");
            return err(ErrorType::INVALID_STORAGE_INDEX);
        }
        break;
    }
    case WaitableEntity::SUBSCRIBER: {
        if (auto result = get_stored_listener<SubscriberListener>(storage_index); result.has_value()) {
            auto& listener_details = result.value();
            if (auto result = drain_events(listener_details->listener); result.has_error()) {
                return err(result.error());
            }
        } else {
            RMW_IOX2_CHAIN_ERROR_MSG("unable to find subscriber listener at provided index");
            return err(ErrorType::INVALID_STORAGE_INDEX);
        }
        break;
    }
    default:
        RMW_IOX2_CHAIN_ERROR_MSG("received trigger for unknown waitable type");
        return err(ErrorType::INVALID_WAITABLE_TYPE);
    }
    return ok();
}

} // namespace rmw::iox2
