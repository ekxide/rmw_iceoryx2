// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/graph_change_handler.hpp"

#include <utility>

namespace rmw::iox2
{

GraphChangeHandler::Registration::Registration(std::shared_ptr<GraphChangeHandler> handler)
    : m_handler{std::move(handler)}
    , m_id{s_next_id++} {
}

GraphChangeHandler::Registration::Registration(Registration&& other) noexcept
    : m_handler{std::move(other.m_handler)}
    , m_id{other.m_id} {
}

auto GraphChangeHandler::Registration::operator=(Registration&& other) noexcept -> Registration& {
    if (this != &other) {
        if (m_handler) {
            m_handler->remove(m_id);
        }
        m_handler = std::move(other.m_handler);
        m_id = other.m_id;
    }
    return *this;
}

GraphChangeHandler::Registration::~Registration() {
    if (m_handler) {
        m_handler->remove(m_id);
    }
}

GraphChangeHandler::GraphChangeHandler(GraphListener graph_listener)
    : m_graph_listener{std::move(graph_listener)} {
}

auto GraphChangeHandler::file_descriptor() const -> ::iox2::FileDescriptorView {
    return m_graph_listener.file_descriptor();
}

auto GraphChangeHandler::register_callback(OnGraphChange callback) -> Registration {
    Registration registration{shared_from_this()};
    std::lock_guard<std::mutex> lock{m_mutex};
    m_callbacks.emplace(registration.m_id, std::move(callback));
    return registration;
}

void GraphChangeHandler::remove(CallbackId id) {
    std::lock_guard<std::mutex> lock{m_mutex};
    m_callbacks.erase(id);
}

void GraphChangeHandler::run_callbacks() {
    std::lock_guard<std::mutex> lock{m_mutex};
    if (!m_graph_listener.drain()) {
        return;
    }
    for (auto& entry : m_callbacks) {
        entry.second();
    }
}

} // namespace rmw::iox2
