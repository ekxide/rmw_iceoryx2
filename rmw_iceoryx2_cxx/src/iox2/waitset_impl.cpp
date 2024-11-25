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
    using ::iox2::WaitSetBuilder;

    auto waitset = WaitSetBuilder().template create<ServiceType::Ipc>();
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

    if (auto result = create_listener(guard_condition.service_name()); result.has_error()) {
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

    if (auto result = create_listener(subscriber.service_name()); result.has_error()) {
        return err(result.error());
    } else {
        auto storage_index = result.value();
        stage_listener(WaitableType::SUBSCRIPTION, storage_index, rmw_index);
        return ok();
    }
}

auto WaitSetImpl::detach_all() -> void {
    m_staged_waitables.clear();
}

auto WaitSetImpl::create_listener(const std::string& name) -> iox::expected<StorageIndex, WaitSetError> {
    using ::iox::err;
    using ::iox::ok;

    auto it = std::find_if(m_listener_storage.begin(), m_listener_storage.end(), [&name](const auto& listener) {
        return listener.service_name == name;
    });
    if (it == m_listener_storage.end()) {
        // An iceoryx2 listener does not exist. Create it.
        auto service_name = ServiceName::create(name.c_str());
        if (service_name.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(service_name.error()));
            return err(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        }

        auto service = m_context.iox2()->service_builder(service_name.value()).event().open_or_create();
        if (service.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(service_name.error()));
            return err(ErrorType::SERVICE_CREATION_FAILURE);
        }

        auto listener = service.value().listener_builder().create();
        if (listener.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(listener.error()));
            return err(ErrorType::LISTENER_CREATION_FAILURE);
        }

        m_listener_storage.emplace_back(ListenerStorage{name, std::move(listener.value())});
        auto storage_index = static_cast<StorageIndex>(m_listener_storage.size() - 1);
        return ok(storage_index);
    } else {
        // An iceoryx2 listener already exists. Reuse it and mark it for attachment.
        auto storage_index = static_cast<StorageIndex>(it - m_listener_storage.begin());
        return ok(storage_index);
    }
}

auto WaitSetImpl::stage_listener(WaitableType waitable_type, StorageIndex storage_index, RmwIndex rmw_index) -> void {
    auto it = std::find_if(m_staged_waitables.begin(), m_staged_waitables.end(), [storage_index](const auto& waitable) {
        return waitable.storage_index == storage_index;
    });

    if (it == m_staged_waitables.end()) {
        m_staged_waitables.push_back(StagedWaitable{storage_index, waitable_type, rmw_index});
    }
}

auto WaitSetImpl::get_listener(StorageIndex storage_index) -> iox::optional<ListenerStorage*> {
    if (storage_index < m_listener_storage.size()) {
        return &m_listener_storage[storage_index];
    }
    return iox::nullopt;
}

auto WaitSetImpl::wait(const iox::optional<Duration>& timeout)
    -> iox::expected<iox::optional<TriggeredWaitable>, ErrorType> {
    using ::iox::err;
    using ::iox::ok;
    using ::iox2::CallbackProgression;
    using ::iox2::ServiceType;
    using ::iox2::WaitSetAttachmentId;

    // Attachments only valid for this call.
    // At end of scope, these are cleaned up thus detached from the waitset.
    iox::optional<WaitSetAttachmentStorage> attached_timeout{};
    std::vector<WaitSetAttachmentStorage> attached_listeners{};
    iox::optional<TriggeredWaitable> triggered_waitable{};

    // Attach timeout if specified.
    if (timeout.has_value()) {
        if (timeout.value() != Duration::zero()) {
            auto guard = m_waitset->attach_interval(timeout.value());
            if (guard.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(guard.error()));
                return err(ErrorType::ATTACHMENT_FAILURE);
            }
            attached_timeout.emplace(WaitSetAttachmentStorage{std::move(guard.value())});
        }
    }

    // Attach all staged listeners.
    for (const auto& staged : m_staged_waitables) {
        if (auto result = get_listener(staged.storage_index); result.has_value()) {
            auto& storage = result.value();
            auto guard = m_waitset->attach_notification(storage->listener);
            if (guard.has_error()) {
                RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(guard.error()));
                return err(ErrorType::ATTACHMENT_FAILURE);
            }
            attached_listeners.emplace_back(WaitSetAttachmentStorage{std::move(guard.value()), staged});
        } else {
            // Storage index was invalid.
            // TODO: Propagate error... or change the implementation to make this impossible.
        }
    }

    // Callbacks
    auto timed_out = [&attached_timeout](AttachmentId& id) -> bool {
        if (attached_timeout.has_value() && attached_timeout->id() == id) {
            return true;
        }
        return false;
    };
    auto triggered = [this, &attached_listeners, &triggered_waitable](AttachmentId& id) -> bool {
        for (size_t i = 0; i < attached_listeners.size(); i++) {
            auto& attachment_id = attached_listeners[i].id();
            if (id == attachment_id) {
                auto& staged_waitable = m_staged_waitables[i];
                auto& listener = m_listener_storage[staged_waitable.storage_index].listener;

                auto result = listener.try_wait_all([&staged_waitable, &triggered_waitable](auto) {
                    if (triggered_waitable.has_value()) {
                        return;
                    }
                    triggered_waitable.emplace(
                        TriggeredWaitable{staged_waitable.waitable_type, staged_waitable.rmw_index});
                });
                if (result.has_error()) {
                    // TODO: Propagate error
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
