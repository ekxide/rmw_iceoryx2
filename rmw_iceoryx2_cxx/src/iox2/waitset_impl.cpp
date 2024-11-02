// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/waitset_impl.hpp"

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
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(waitset.error()));
        error.emplace(ErrorType::WAITSET_CREATION_FAILURE);
        return;
    }
    m_waitset.emplace(std::move(waitset.value()));
}

auto WaitSetImpl::attach(GuardConditionImpl& guard_condition) -> iox::expected<void, WaitSetError> {
    return attach_listener(guard_condition.service_name());
}

auto WaitSetImpl::attach(SubscriberImpl& subscriber) -> iox::expected<void, WaitSetError> {
    return attach_listener(subscriber.service_name());
}

auto WaitSetImpl::wait(const Duration& timeout) -> iox::expected<void, ErrorType> {
    using ::iox::err;
    using ::iox::ok;
    using ::iox2::ServiceType;
    using ::iox2::WaitSetAttachmentId;

    // Timeout is only valid for this call. Will be cleaned up and removed automatically at end of scope.
    iox::optional<Guard> timeout_guard;
    iox::optional<AttachmentId> timeout_id;
    auto on_timeout = [this, &timeout_id](AttachmentId& id) {
        if (timeout_id && timeout_id.value() == id) {
            m_waitset->stop();
        }
    };
    if (timeout != Duration::zero()) {
        auto attachment = m_waitset->attach_interval(timeout);
        if (attachment.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(attachment.error()));
            return err(ErrorType::ATTACHMENT_FAILURE);
        }
        timeout_guard.emplace(std::move(attachment.value()));
        timeout_id.emplace(AttachmentId::from_guard(timeout_guard.value()));
    }

    auto result = m_waitset->wait_and_process([this, &on_timeout](auto id) {
        on_timeout(id);
        on_trigger(id);
    });

    return ok();
}

auto WaitSetImpl::attach_listener(const std::string& name) -> iox::expected<void, WaitSetError> {
    using ::iox::err;
    using ::iox::ok;

    if (std::find_if(m_attached_listeners.begin(),
                     m_attached_listeners.end(),
                     [&name](const auto& attached) { return attached.service_name == name; })
        == m_attached_listeners.end()) {
        auto service_name = ServiceName::create(name.c_str());
        if (service_name.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(service_name.error()));
            return err(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        }

        auto service = m_context.node().as_iox2().service_builder(service_name.value()).event().open_or_create();
        if (service.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(service_name.error()));
            return err(ErrorType::SERVICE_CREATION_FAILURE);
        }

        auto listener = service.value().listener_builder().create();
        if (listener.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(listener.error()));
            return err(ErrorType::LISTENER_CREATION_FAILURE);
        }

        auto guard = m_waitset->attach_notification(listener.value());
        if (guard.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox2::error_string(guard.error()));
            return err(ErrorType::ATTACHMENT_FAILURE);
        }

        auto attachment_id = AttachmentId::from_guard(guard.value());

        m_attached_listeners.push_back(
            AttachedListener{name, std::move(attachment_id), std::move(guard.value()), std::move(listener.value())});
    }

    return ok();
}

auto WaitSetImpl::on_trigger(AttachmentId& id) -> void {
    auto it = std::find_if(m_attached_listeners.begin(), m_attached_listeners.end(), [&id](const auto& attached) {
        return attached.id == id;
    });
    if (it != m_attached_listeners.end()) {
        auto& listener = it->listener;
        if (listener
                .try_wait_all([this](auto) {
                    // TODO: What to do when getting a trigger?
                    this->m_waitset->stop();
                })
                .has_error()) {
            // TODO: Propagate error
        };
    }
}

} // namespace rmw::iox2
