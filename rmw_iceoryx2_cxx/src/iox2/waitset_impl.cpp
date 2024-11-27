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
#include "rmw_iceoryx2_cxx/error_handling.hpp"

namespace rmw::iox2
{

WaitSetImpl::WaitSetImpl(CreationLock, iox::optional<WaitSetError>& error, ContextImpl& context)
    : m_context{context} {
    using ::iox2::ServiceType;

    auto waitset = Iceoryx2::WaitSet::Builder().template create<ServiceType::Ipc>();
    if (waitset.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(waitset.error()));
        error.emplace(ErrorType::WAITSET_CREATION_FAILURE);
        return;
    }
    m_waitset.emplace(std::move(waitset.value()));
}

auto WaitSetImpl::attach(RmwIndex rmw_index, GuardConditionImpl& guard_condition) -> iox::expected<void, WaitSetError> {
    using ::iox::err;
    using ::iox::ok;

    if (auto result = create_listener<GuardConditionListener>(guard_condition.service_name()); result.has_error()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        stage_listener(WaitableType::GUARD_CONDITION, storage_index, rmw_index);
        return ok();
    }
}

auto WaitSetImpl::attach(RmwIndex rmw_index, SubscriberImpl& subscriber) -> iox::expected<void, WaitSetError> {
    using ::iox::err;
    using ::iox::ok;

    if (auto result = create_listener<SubscriberListener>(subscriber.service_name()); result.has_error()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        stage_listener(WaitableType::SUBSCRIBER, storage_index, rmw_index);
        return ok();
    }
}

auto WaitSetImpl::detach_all() -> void {
    m_staged_waitables.clear();
}

auto WaitSetImpl::stage_listener(WaitableType waitable_type, StorageIndex storage_index, RmwIndex rmw_index) -> void {
    auto it = std::find_if(
        m_staged_waitables.begin(), m_staged_waitables.end(), [waitable_type, storage_index](const auto& staged) {
            if (staged.waitable_type == waitable_type && staged.storage_index == storage_index) {
                return true;
            }
            return false;
        });

    if (it == m_staged_waitables.end()) {
        m_staged_waitables.push_back(StagedWaitable{storage_index, waitable_type, rmw_index});
    }
}

auto WaitSetImpl::wait(const iox::optional<Duration>& timeout)
    -> iox::expected<iox::optional<TriggeredWaitable>, ErrorType> {
    using ::iox::err;
    using ::iox::ok;
    using ::iox2::CallbackProgression;

    // Attachments only valid for this call.
    // At end of scope, these are cleaned up thus detached from the waitset.
    iox::optional<AttachmentDetails> attached_timeout{};
    std::vector<AttachmentDetails> attached_listeners{};
    iox::optional<TriggeredWaitable> triggered_waitable{};

    // Attach timeout if specified.
    if (timeout.has_value()) {
        if (timeout.value() != Duration::zero()) {
            auto guard = m_waitset->attach_interval(timeout.value());
            if (guard.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(guard.error()));
                return err(ErrorType::ATTACHMENT_FAILURE);
            }
            attached_timeout.emplace(std::move(guard.value()));
        }
    }

    // Attach all staged listeners.
    // TODO: optimize
    for (const auto& staged : m_staged_waitables) {
        if (staged.waitable_type == WaitableType::GUARD_CONDITION) {
            if (auto result = get_stored_listener<GuardConditionListener>(staged.storage_index); result.has_value()) {
                auto& storage = result.value();
                auto guard = m_waitset->attach_notification(storage->listener.file_descriptor());
                if (guard.has_error()) {
                    RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(guard.error()));
                    return err(ErrorType::ATTACHMENT_FAILURE);
                }
                attached_listeners.emplace_back(std::move(guard.value()), staged);
            } else {
                // Storage index was invalid.
                // TODO: Propagate error... or change the implementation to make this impossible.
            }
        } else if (staged.waitable_type == WaitableType::SUBSCRIBER) {
            if (auto result = get_stored_listener<SubscriberListener>(staged.storage_index); result.has_value()) {
                auto& storage = result.value();
                auto guard = m_waitset->attach_notification(storage->listener.file_descriptor());
                if (guard.has_error()) {
                    RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(guard.error()));
                    return err(ErrorType::ATTACHMENT_FAILURE);
                }
                attached_listeners.emplace_back(std::move(guard.value()), staged);
            } else {
                // Storage index was invalid.
                // TODO: Propagate error... or change the implementation to make this impossible.
            }
        } else {
            // TODO: panic
        }
    }

    // Callbacks
    auto timed_out = [&attached_timeout](auto& id) -> bool {
        if (attached_timeout.has_value() && attached_timeout->id() == id) {
            return true;
        }
        return false;
    };

    // TODO: optimize
    auto triggered = [this, &attached_listeners, &triggered_waitable](auto& id) -> bool {
        for (size_t i = 0; i < attached_listeners.size(); i++) {
            auto& attached_listener = attached_listeners[i];
            auto& attachment_id = attached_listener.id();
            // If ID matches, this staged waitable is definitely a GuardCondition
            if (id == attachment_id) {
                auto& staged = m_staged_waitables[i];
                if (staged.waitable_type == WaitableType::GUARD_CONDITION) {
                    if (auto result = get_stored_listener<GuardConditionListener>(staged.storage_index);
                        result.has_value()) {
                        auto& stored = result.value();
                        if (stored->listener
                                .try_wait_all([&staged, &triggered_waitable](auto) {
                                    if (triggered_waitable.has_value()) {
                                        // Already triggered
                                        return;
                                    }
                                    triggered_waitable.emplace(
                                        TriggeredWaitable{staged.waitable_type, staged.rmw_index});
                                })
                                .has_error()) {
                        }
                        {
                            // TODO: Propagate error via expected
                        }
                    }
                } else if (staged.waitable_type == WaitableType::SUBSCRIBER) {
                    if (auto result = get_stored_listener<SubscriberListener>(staged.storage_index);
                        result.has_value()) {
                        auto& stored = result.value();
                        if (stored->listener
                                .try_wait_all([&staged, &triggered_waitable](auto) {
                                    if (triggered_waitable.has_value()) {
                                        // Already triggered
                                        return;
                                    }
                                    triggered_waitable.emplace(
                                        TriggeredWaitable{staged.waitable_type, staged.rmw_index});
                                })
                                .has_error()) {
                        }
                        {
                            // TODO: Propagate error via expected
                        }
                    }
                }
            }
        }
        return triggered_waitable.has_value();
    };

    // Wait
    auto on_event = [&timed_out, &triggered](auto id) -> CallbackProgression {
        if (triggered(id) || timed_out(id)) {
            return CallbackProgression::Stop;
        }
        return CallbackProgression::Continue;
    };

    // If timeout explicitly set to zero, don't wait, just process pending triggers.
    auto result = timeout.has_value() && timeout.value() != Duration::zero()
                      ? m_waitset->wait_and_process(on_event)
                      : m_waitset->wait_and_process_once(on_event);
    if (result.has_error()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(result.error()));
        return err(ErrorType::WAIT_FAILURE);
    }

    return ok(triggered_waitable);
}

} // namespace rmw::iox2
