// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"

#include "iox2/bb/detail/assertions.hpp"
#include "iox2/bb/into.hpp"
#include "rcutils/error_handling.h"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"

namespace
{

auto open_graph_service(::rmw::iox2::Iceoryx2& iox2, const ::rmw::iox2::Iceoryx2::ServiceName& name)
    -> ::iox2::bb::Expected<::iox2::PortFactoryEvent<::iox2::ServiceType::Ipc>, ::iox2::EventOpenOrCreateError> {
    auto opened = iox2.ipc().service_builder(name).event().open();
    if (opened.has_value()) {
        return std::move(opened.value());
    }
    return iox2.ipc()
        .service_builder(name)
        .event()
        .max_nodes(::rmw::iox2::GRAPH_MAX_CONTEXTS)
        .max_notifiers(::rmw::iox2::GRAPH_MAX_GUARD_CONDITIONS)
        .max_listeners(::rmw::iox2::GRAPH_MAX_GUARD_CONDITIONS)
        .open_or_create();
}

} // namespace

rmw_context_impl_s::rmw_context_impl_s(CreationLock,
                                       ::iox2::bb::Optional<ErrorType>& error,
                                       const uint32_t id,
                                       const rmw_init_options_impl_s& options)
    : m_id{id}
    , m_options{options} {
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::GraphGuardCondition;
    namespace names = rmw::iox2::names;

    if (auto result = create_in_place<Iceoryx2>(m_iox2, names::context(id)); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create Handle");
        error.emplace(ErrorType::HANDLE_CREATION_FAILURE);
        return;
    }

    auto graph_service_name = Iceoryx2::ServiceName::create(names::graph().c_str());
    if (!graph_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(graph_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    auto graph_service = open_graph_service(m_iox2.value(), graph_service_name.value());
    if (!graph_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(graph_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }
    m_graph_service.emplace(std::move(graph_service.value()));

    if (auto result = create_in_place<GraphGuardCondition>(m_graph_guard_condition, m_graph_service.value());
        !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the graph guard condition of the context");
        error.emplace(ErrorType::GRAPH_GUARD_CONDITION_CREATION_FAILURE);
        return;
    }
}

rmw_context_impl_s::rmw_context_impl_s(rmw_context_impl_s&& other) noexcept
    : m_id{other.m_id}
    , m_iox2{std::move(other.m_iox2)}
    , m_graph_service{std::move(other.m_graph_service)}
    , m_graph_guard_condition{std::move(other.m_graph_guard_condition)}
    , m_options{std::move(other.m_options)}
    , m_guard_condition_counter{other.m_guard_condition_counter.exchange(0)} {
}

auto rmw_context_impl_s::operator=(rmw_context_impl_s&& other) noexcept -> rmw_context_impl_s& {
    if (this != &other) {
        m_id = other.m_id;
        m_iox2 = std::move(other.m_iox2);
        m_graph_service = std::move(other.m_graph_service);
        m_graph_guard_condition = std::move(other.m_graph_guard_condition);
        m_options = std::move(other.m_options);
        m_guard_condition_counter.store(other.m_guard_condition_counter.exchange(0));
    }
    return *this;
}

auto rmw_context_impl_s::id() -> uint32_t {
    return m_id;
}

auto rmw_context_impl_s::iox2() -> Iceoryx2& {
    return m_iox2.value();
}

auto rmw_context_impl_s::options() const -> const rmw_init_options_impl_s& {
    return m_options;
}

auto rmw_context_impl_s::generate_guard_condition_id() -> uint32_t {
    return m_guard_condition_counter++;
}

auto rmw_context_impl_s::graph_service() -> ::iox2::bb::Optional<GraphService>& {
    return m_graph_service;
}

auto rmw_context_impl_s::notify_graph_change() -> void {
    if (!m_graph_guard_condition.has_value()) {
        IOX2_PANIC("Graph guard condition is missing: the context was moved-from or used after a failed construction");
    }
    if (!m_graph_guard_condition->trigger().has_value()) {
        RMW_IOX2_LOG_WARN("failed to notify a graph change: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
}
