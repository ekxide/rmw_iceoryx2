// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/event_id.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"

namespace rmw::iox2
{

GuardCondition::GuardCondition(CreationLock,
                               ::iox2::bb::Optional<ErrorType>& error,
                               Context& context,
                               GuardConditionKind kind)
    : m_trigger_id{context.generate_guard_condition_id()}
    , m_service_name{names::guard_condition(context.id(), m_trigger_id)} {
    auto iox2_service_name = Iceoryx2::ServiceName::create(m_service_name.c_str());
    if (!iox2_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    auto iox2_service = context.iox2().local().service_builder(iox2_service_name.value()).event().open_or_create();
    if (!iox2_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_notifier = iox2_service.value().notifier_builder().create();
    if (!iox2_notifier.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_notifier.error()));
        error.emplace(ErrorType::NOTIFIER_CREATION_FAILURE);
        return;
    }

    m_iox2_notifier.emplace(std::move(iox2_notifier.value()));

    auto iox2_listener = iox2_service.value().listener_builder().create();
    if (!iox2_listener.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_listener.error()));
        error.emplace(ErrorType::LISTENER_CREATION_FAILURE);
        return;
    }

    m_iox2_listener.emplace(std::move(iox2_listener.value()));

    if (kind == GuardConditionKind::USER) {
        return;
    }

    auto graph_listener = context.graph_service()->listener_builder().create();
    if (!graph_listener.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(graph_listener.error()));
        error.emplace(ErrorType::LISTENER_CREATION_FAILURE);
        return;
    }
    m_iox2_graph_listener.emplace(std::move(graph_listener.value()));
};

auto GuardCondition::trigger_id() const -> uint32_t {
    return m_trigger_id;
}

auto GuardCondition::unique_id() -> const ::iox2::bb::Optional<RawIdType>& {
    auto& bytes = m_iox2_unique_id->bytes();
    return bytes;
}


auto GuardCondition::service_name() const -> const std::string& {
    return m_service_name;
}

auto GuardCondition::trigger() -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    if (auto result = m_iox2_notifier->notify(); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    };

    return {};
}

auto GuardCondition::drain() -> bool {
    bool triggered = false;
    auto on_event = [&triggered](auto) { triggered = true; };
    (void)m_iox2_listener->try_wait(on_event);
    if (m_iox2_graph_listener.has_value()) {
        (void)m_iox2_graph_listener->try_wait(on_event);
    }
    return triggered;
}

auto GuardCondition::file_descriptor() const -> ::iox2::FileDescriptorView {
    if (m_iox2_graph_listener.has_value()) {
        return m_iox2_graph_listener->file_descriptor();
    }
    return m_iox2_listener->file_descriptor();
}

} // namespace rmw::iox2
