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

rmw_context_impl_s::rmw_context_impl_s(CreationLock,
                                       ::iox2::bb::Optional<ErrorType>& error,
                                       const uint32_t id,
                                       const rmw_init_options_impl_s& options)
    : m_id{id}
    , m_options{options} {
    using ::rmw::iox2::create;
    using ::rmw::iox2::create_in_place;
    using ::rmw::iox2::GraphGuardCondition;
    namespace names = rmw::iox2::names;

    if (auto result = create_in_place<Iceoryx2>(m_iox2, names::context(id)); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create Handle");
        error.emplace(ErrorType::HANDLE_CREATION_FAILURE);
        return;
    }

    if (auto result = create_in_place<GraphGuardCondition>(m_graph_guard_condition, m_iox2.value());
        !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the graph guard condition of the context");
        error.emplace(ErrorType::GRAPH_GUARD_CONDITION_CREATION_FAILURE);
        return;
    }

    if (!m_callback_thread->start().has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to start the callback thread of the context");
        error.emplace(ErrorType::CALLBACK_THREAD_CREATION_FAILURE);
        return;
    }

    auto graph_listener = create<::rmw::iox2::GraphListener>(m_iox2.value());
    if (!graph_listener.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the graph listener of the graph change handler");
        error.emplace(ErrorType::GRAPH_CHANGE_HANDLER_CREATION_FAILURE);
        return;
    }
    m_graph_change_handler = std::make_shared<::rmw::iox2::GraphChangeHandler>(std::move(graph_listener.value()));
    auto graph_change_callback_registration = m_callback_thread->add(
        m_graph_change_handler->file_descriptor(), [handler = m_graph_change_handler] { handler->run_callbacks(); });
    if (!graph_change_callback_registration.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to add the graph change handler to the callback thread");
        error.emplace(ErrorType::GRAPH_CHANGE_HANDLER_CREATION_FAILURE);
        return;
    }
    m_graph_change_callback_registration.emplace(std::move(graph_change_callback_registration.value()));
}

rmw_context_impl_s::rmw_context_impl_s(rmw_context_impl_s&& other) noexcept
    : m_id{other.m_id}
    , m_iox2{std::move(other.m_iox2)}
    , m_graph_guard_condition{std::move(other.m_graph_guard_condition)}
    , m_options{std::move(other.m_options)}
    , m_callback_thread{std::move(other.m_callback_thread)}
    , m_graph_change_handler{std::move(other.m_graph_change_handler)}
    , m_graph_change_callback_registration{std::move(other.m_graph_change_callback_registration)} {
}

auto rmw_context_impl_s::operator=(rmw_context_impl_s&& other) noexcept -> rmw_context_impl_s& {
    if (this != &other) {
        m_id = other.m_id;
        m_iox2 = std::move(other.m_iox2);
        m_graph_guard_condition = std::move(other.m_graph_guard_condition);
        m_options = std::move(other.m_options);
        m_callback_thread = std::move(other.m_callback_thread);
        m_graph_change_handler = std::move(other.m_graph_change_handler);
        m_graph_change_callback_registration = std::move(other.m_graph_change_callback_registration);
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

auto rmw_context_impl_s::notify_graph_change() -> void {
    if (!m_graph_guard_condition.has_value()) {
        IOX2_PANIC("Graph guard condition is missing: the context was moved-from or used after a failed construction");
    }
    if (!m_graph_guard_condition->trigger().has_value()) {
        RMW_IOX2_LOG_WARN("failed to notify a graph change: %s", rcutils_get_error_string().str);
        rcutils_reset_error();
    }
}

auto rmw_context_impl_s::callback_thread() const -> const std::shared_ptr<::rmw::iox2::CallbackThread>& {
    return m_callback_thread;
}

auto rmw_context_impl_s::graph_change_handler() const -> const std::shared_ptr<::rmw::iox2::GraphChangeHandler>& {
    return m_graph_change_handler;
}
