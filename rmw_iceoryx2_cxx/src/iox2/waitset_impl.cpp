// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/waitset_impl.hpp"
#include "iox2/service_name.hpp"
#include "iox2/waitset.hpp"

namespace rmw::iox2
{

WaitSetImpl::WaitSetImpl(ContextImpl& context)
    : m_context{context} {
    using ::iox2::ServiceType;
    using ::iox2::WaitSetBuilder;

    auto waitset = WaitSetBuilder().template create<ServiceType::Ipc>().expect("TODO: propagate");
    m_waitset.emplace(std::move(waitset));
}

auto WaitSetImpl::attach(GuardConditionImpl& guard_condition, Callback callback) -> void {
    using ::iox2::ServiceName;


    auto service_name = guard_condition.service_name();
    if (std::find_if(m_attached_listeners.begin(),
                     m_attached_listeners.end(),
                     [&service_name](const auto& attached) { return attached.service_name == service_name; })
        == m_attached_listeners.end()) {
        auto service = m_context.node()
                           .as_iox2()
                           .service_builder(ServiceName::create(service_name.c_str()).expect("TODO: propagate"))
                           .event()
                           .open_or_create()
                           .expect("TODO: propagate");
        auto listener = service.listener_builder().create().expect("TODO: propagate");
        auto guard = m_waitset->attach_notification(listener).expect("TODO: propagate");
        auto attachment_id = AttachmentId::from_guard(guard);

        m_attached_listeners.push_back(
            AttachedListener{service_name, std::move(attachment_id), std::move(guard), std::move(listener)});
    }
}

auto WaitSetImpl::attach(SubscriberImpl& subscriber) -> void {
}

auto WaitSetImpl::wait(const Duration& timeout) -> void {
    using ::iox2::ServiceType;
    using ::iox2::WaitSetAttachmentId;

    auto on_trigger = [this](AttachmentId& id) {
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
    };

    // Timeout is only valid for this call. Will be cleaned up and removed automatically at end of scope.
    iox::optional<Guard> timeout_guard;
    iox::optional<AttachmentId> timeout_id;
    auto on_timeout = [this, &timeout_id](AttachmentId& id) {
        if (timeout_id && timeout_id.value() == id) {
            m_waitset->stop();
        }
    };
    if (timeout != Duration::zero()) {
        timeout_guard.emplace(m_waitset->attach_interval(timeout).expect("TODO: propagate"));
        timeout_id.emplace(AttachmentId::from_guard(timeout_guard.value()));
    }

    auto result = m_waitset->wait_and_process([&on_trigger, &on_timeout](auto id) {
        on_timeout(id);
        on_trigger(id);
    });
}

} // namespace rmw::iox2
