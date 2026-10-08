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
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"

#include <array>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <utility>

namespace rmw::iox2
{

UserGuardCondition::UserGuardCondition(CreationLock, ::iox2::bb::Optional<ErrorType>& error) {
    std::array<int, 2> pipe_ends{};
    if (::pipe(pipe_ends.data()) != 0) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the pipe of the guard condition");
        error.emplace(ErrorType::PIPE_CREATION_FAILURE);
        return;
    }

    m_read_end = pipe_ends[0];
    m_write_end = pipe_ends[1];

    if (::fcntl(m_read_end, F_SETFL, O_NONBLOCK) != 0 || ::fcntl(m_write_end, F_SETFL, O_NONBLOCK) != 0) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to make the pipe of the guard condition non-blocking");
        error.emplace(ErrorType::PIPE_CREATION_FAILURE);
        return;
    }

    m_read_end_view = ::iox2::FileDescriptor::create_non_owning(m_read_end);
}

UserGuardCondition::UserGuardCondition(UserGuardCondition&& other) noexcept
    : m_read_end{std::exchange(other.m_read_end, -1)}
    , m_write_end{std::exchange(other.m_write_end, -1)}
    , m_read_end_view{std::move(other.m_read_end_view)} {
}

auto UserGuardCondition::operator=(UserGuardCondition&& other) noexcept -> UserGuardCondition& {
    std::swap(m_read_end, other.m_read_end);
    std::swap(m_write_end, other.m_write_end);
    std::swap(m_read_end_view, other.m_read_end_view);
    return *this;
}

UserGuardCondition::~UserGuardCondition() {
    if (m_read_end >= 0) {
        ::close(m_read_end);
    }
    if (m_write_end >= 0) {
        ::close(m_write_end);
    }
}

auto UserGuardCondition::trigger() -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    const uint8_t token = 1;
    if (::write(m_write_end, &token, sizeof(token)) < 0 && errno != EAGAIN) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to write to the pipe of the guard condition");
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return {};
}

auto UserGuardCondition::drain() -> bool {
    bool triggered = false;
    std::array<uint8_t, 64> tokens{};
    while (::read(m_read_end, tokens.data(), tokens.size()) > 0) {
        triggered = true;
    }
    return triggered;
}

auto UserGuardCondition::file_descriptor() const -> ::iox2::FileDescriptorView {
    return m_read_end_view->as_view();
}

GraphGuardCondition::GraphGuardCondition(CreationLock, ::iox2::bb::Optional<ErrorType>& error, Iceoryx2& iox2) {
    auto graph_service_name = Iceoryx2::ServiceName::create(names::graph().c_str());
    if (!graph_service_name.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(graph_service_name.error()));
        error.emplace(ErrorType::SERVICE_NAME_CREATION_FAILURE);
        return;
    }

    auto graph_service = iox2.open_or_create_event_service(
        graph_service_name.value(), GRAPH_MAX_GUARD_CONDITIONS, GRAPH_MAX_GUARD_CONDITIONS, GRAPH_MAX_GUARD_CONDITIONS);
    if (!graph_service.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(graph_service.error()));
        error.emplace(ErrorType::SERVICE_CREATION_FAILURE);
        return;
    }

    auto iox2_notifier = graph_service->notifier_builder().create();
    if (!iox2_notifier.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_notifier.error()));
        error.emplace(ErrorType::NOTIFIER_CREATION_FAILURE);
        return;
    }
    m_iox2_notifier.emplace(std::move(iox2_notifier.value()));

    auto iox2_listener = graph_service->listener_builder().create();
    if (!iox2_listener.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(iox2_listener.error()));
        error.emplace(ErrorType::LISTENER_CREATION_FAILURE);
        return;
    }
    m_iox2_listener.emplace(std::move(iox2_listener.value()));
}

auto GraphGuardCondition::trigger() -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    if (auto result = m_iox2_notifier->notify(); !result.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(result.error()));
        return err(ErrorType::NOTIFICATION_FAILURE);
    }

    return {};
}

auto GraphGuardCondition::drain() -> bool {
    bool triggered = false;
    (void)m_iox2_listener->try_wait([&triggered](auto) { triggered = true; });
    return triggered;
}

auto GraphGuardCondition::file_descriptor() const -> ::iox2::FileDescriptorView {
    return m_iox2_listener->file_descriptor();
}

} // namespace rmw::iox2
