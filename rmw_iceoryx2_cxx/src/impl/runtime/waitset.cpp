// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/waitset.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/callback_progression.hpp"
#include "iox2/waitset.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"

namespace rmw::iox2
{

WaitSet::WaitSet(CreationLock, ::iox2::bb::Optional<WaitSetError>& error, Context& context)
    : m_context{context} {
    auto waitset = Iceoryx2::WaitSet::create();
    if (!waitset.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(waitset.error()));
        error.emplace(ErrorType::WAITSET_CREATION_FAILURE);
        return;
    }
    m_waitset.emplace(std::move(waitset.value()));
}

auto WaitSet::context() -> Context& {
    return m_context.get();
}

auto WaitSet::map(RmwIndex rmw_index, GuardCondition& guard_condition) -> ::iox2::bb::Expected<void, WaitSetError> {
    using ::iox2::bb::err;

    if (auto result = get_storage_index<GuardConditionListener>(guard_condition.service_name()); !result.has_value()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        map_stored_listener(WaitableEntity::GUARD_CONDITION, storage_index, rmw_index);
        return {};
    }
}

auto WaitSet::map(RmwIndex rmw_index, Subscriber& subscriber) -> ::iox2::bb::Expected<void, WaitSetError> {
    using ::iox2::bb::err;

    if (auto result = get_storage_index<SubscriberListener>(subscriber.service_name()); !result.has_value()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        map_stored_listener(WaitableEntity::SUBSCRIBER, storage_index, rmw_index);
        return {};
    }
}

auto WaitSet::map_stored_listener(WaitableEntity waitable_type,
                                  StorageIndex storage_index,
                                  RmwIndex rmw_index) -> void {
    auto it = std::find_if(m_mapping.begin(), m_mapping.end(), [waitable_type, storage_index](const auto& staged) {
        return staged.waitable_type == waitable_type && staged.storage_index == storage_index;
    });

    if (it == m_mapping.end()) {
        m_mapping.push_back(RmwMapping{waitable_type, storage_index, rmw_index});
    }
}

auto WaitSet::unmap_all() -> void {
    // Detaching removes the mapping, but the listener remains in the storage for re-use in subsequent calls.
    m_mapping.clear();
}

auto WaitSet::wait(const ::iox2::bb::Optional<Duration>& timeout)
    -> ::iox2::bb::Expected<std::vector<TriggeredWaitable>, ErrorType> {
    using ::iox2::CallbackProgression;
    using ::iox2::bb::err;

    if (m_mapping.empty()) {
        if (zero_timeout(timeout)) {
            // This is a NOOP.
            return std::vector<TriggeredWaitable>{};
        }
        if (no_timeout(timeout)) {
            // Trying to wait indefinitely with nothing mapped.
            // This would deadlock.
            return err(ErrorType::WAIT_FAILURE);
        }
    }

    // Context for this specific wait call.
    // Cleaned up automatically at end of scope, detaching all attachments from the waitset.
    WaitContext ctx;

    // Attach the timeout to the waitset
    if (timeout.has_value()) {
        if (auto result = attach_timeout(timeout.value(), ctx); !result.has_value()) {
            return err(result.error());
        }
    }

    // Attached all previously mapped listeners
    if (auto result = attach_mapped_listeners(ctx); !result.has_value()) {
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
                    !result.has_value()) {
                    RMW_IOX2_LOG_ERROR("Failed to process trigger from a waitset attachment");
                    // Continue checking for other triggers even on error
                    return CallbackProgression::Continue;
                } else {
                    ctx.result.push_back(TriggeredWaitable{attachment.mapping()});
                    // Continue checking for other triggers after finding one
                    return CallbackProgression::Continue;
                }
            }
        }

        RMW_IOX2_LOG_ERROR("Waitset was triggered by an unmapped subscriber or guard condition");
        // Continue looking for notifications from other attachments so as not to hinder functionality.
        return CallbackProgression::Continue;
    };

    // If timeout is non-zero, block and wait, otherwise check for events and return immediately.
    if (auto result = no_timeout(timeout) ? m_waitset->wait_and_process_once(on_event)
                                          : m_waitset->wait_and_process_once_with_timeout(on_event, timeout.value());
        !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::WAIT_FAILURE);
    }

    return std::move(ctx.result);
}

auto WaitSet::zero_timeout(const ::iox2::bb::Optional<Duration>& timeout) const -> bool {
    return timeout.has_value() && timeout.value() == Duration::zero();
}

auto WaitSet::no_timeout(const ::iox2::bb::Optional<Duration>& timeout) const -> bool {
    return !timeout.has_value();
}

auto WaitSet::attach_timeout(const Duration& timeout, WaitContext& ctx) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    auto guard = m_waitset->attach_interval(timeout);
    if (!guard.has_value()) {
        return err(ErrorType::ATTACHMENT_FAILURE);
    }
    ctx.attached_timeout.emplace(std::move(guard.value()));
    return {};
}

auto WaitSet::attach_mapped_listeners(WaitContext& ctx) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    for (const auto& staged : m_mapping) {
        auto result = attach_mapped_listener(staged);
        if (!result.has_value()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to attach mapped listeners to waitset");
            return err(result.error());
        }
        ctx.attached_listeners.push_back(std::move(result.value()));
    }
    return {};
}

auto WaitSet::attach_mapped_listener(const RmwMapping& mapping) -> ::iox2::bb::Expected<AttachmentDetails, ErrorType> {
    using ::iox2::bb::err;

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

auto WaitSet::process_trigger(const WaitableEntity waitable_type,
                              const StorageIndex storage_index) -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    // Drain all events from the trigger.
    // The value nor number of triggers is irrelevant, so no callback logic required.
    auto drain_events = [](auto& listener) -> ::iox2::bb::Expected<void, ErrorType> {
        if (auto result = listener.try_wait_all([&](auto) {}); !result.has_value()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to retrieve events from listener");
            return err(ErrorType::LISTENER_FAILURE);
        }
        return ::iox2::bb::Expected<void, ErrorType>{};
    };

    // Retrieve the listener from the corresponding storage and drain all of its events
    switch (waitable_type) {
    case WaitableEntity::GUARD_CONDITION: {
        if (auto result = get_stored_listener<GuardConditionListener>(storage_index); result.has_value()) {
            auto& listener_details = result.value();
            if (auto result = drain_events(listener_details->listener); !result.has_value()) {
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
            if (auto result = drain_events(listener_details->listener); !result.has_value()) {
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
    return {};
}

} // namespace rmw::iox2
