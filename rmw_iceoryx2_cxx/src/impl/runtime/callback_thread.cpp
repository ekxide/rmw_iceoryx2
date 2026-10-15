// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/callback_thread.hpp"

#include "iox2/bb/into.hpp"
#include "iox2/waitset.hpp"
#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/common/log.hpp"

#include <cstdint>
#include <utility>

namespace rmw::iox2
{

CallbackThread::Registration::Registration(std::shared_ptr<CallbackThread> thread, uint64_t id)
    : m_thread{std::move(thread)}
    , m_id{id} {
}

CallbackThread::Registration::Registration(Registration&& other) noexcept
    : m_thread{std::move(other.m_thread)}
    , m_id{other.m_id} {
}

auto CallbackThread::Registration::operator=(Registration&& other) noexcept -> Registration& {
    if (this != &other) {
        if (m_thread) {
            m_thread->remove(m_id);
        }
        m_thread = std::move(other.m_thread);
        m_id = other.m_id;
    }
    return *this;
}

CallbackThread::Registration::~Registration() {
    if (m_thread) {
        m_thread->remove(m_id);
    }
}

CallbackThread::~CallbackThread() {
    if (!m_thread.joinable()) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock{m_mutex};
        m_stop = true;
    }
    (void)m_wake->trigger();
    m_thread.join();
}

auto CallbackThread::add(Listener& listener, rmw_event_callback_t callback, const void* user_data)
    -> ::iox2::bb::Expected<Registration, ErrorType> {
    auto count_notifications = [&listener]() -> uint64_t {
        uint64_t count = 0;
        if (auto result = listener.try_wait([&count](auto activation) { count += activation.count(); });
            !result.has_value()) {
            RMW_IOX2_LOG_ERROR("failed to retrieve events from listener");
        }
        return count;
    };
    return add(listener.file_descriptor(), count_notifications, callback, user_data);
}

auto CallbackThread::add(::iox2::FileDescriptorView wake_up,
                         CountEvents count_events,
                         rmw_event_callback_t callback,
                         const void* user_data) -> ::iox2::bb::Expected<Registration, ErrorType> {
    using ::iox2::bb::err;

    std::unique_lock<std::mutex> lock{m_mutex};
    const auto id = m_next_id++;
    auto& added_callback =
        m_callbacks.emplace(id, Callback{wake_up, std::move(count_events), callback, user_data, ::iox2::bb::NULLOPT})
            .first->second;
    call(added_callback);
    apply_changes(lock);

    if (m_stop) {
        RMW_IOX2_CHAIN_ERROR_MSG("the callback thread stopped");
        return err(ErrorType::STOPPED);
    }
    if (m_callbacks.count(id) == 0) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to wait on the file descriptor of the callback");
        return err(ErrorType::ATTACHMENT_FAILURE);
    }
    return Registration{shared_from_this(), id};
}

void CallbackThread::remove(uint64_t id) {
    std::unique_lock<std::mutex> lock{m_mutex};
    auto removed = m_callbacks.find(id);
    if (removed == m_callbacks.end()) {
        return;
    }
    auto& removed_callback = removed->second;
    if (removed_callback.guard.has_value()) {
        m_ids.erase(AttachmentId::from_guard(removed_callback.guard.value()));
        m_removed_guards.push_back(std::move(removed_callback.guard.value()));
    }
    m_callbacks.erase(removed);
    apply_changes(lock);
}

auto CallbackThread::start() -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    if (!create_in_place(m_wake).has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the wake-up of the callback thread");
        return err(ErrorType::WAKE_UP_FAILURE);
    }

    auto waitset = Iceoryx2::WaitSet::create();
    if (!waitset.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(waitset.error()));
        return err(ErrorType::WAITSET_CREATION_FAILURE);
    }
    m_waitset.emplace(std::move(waitset.value()));

    auto wake_guard = m_waitset->attach_notification(m_wake->file_descriptor());
    if (!wake_guard.has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG(::iox2::bb::into<const char*>(wake_guard.error()));
        return err(ErrorType::WAKE_UP_FAILURE);
    }
    m_wake_id.emplace(AttachmentId::from_guard(wake_guard.value()));
    m_wake_guard.emplace(std::move(wake_guard.value()));

    m_thread = std::thread([this] { run(); });
    return {};
}

void CallbackThread::apply_changes(std::unique_lock<std::mutex>& lock) {
    m_changed = true;
    (void)m_wake->trigger();
    m_changes_applied.wait(lock, [this] { return !m_changed || m_stop; });
}

void CallbackThread::run() {
    auto result = m_waitset->wait_and_process([this](auto id) { return on_event(id); });
    if (!result.has_value()) {
        RMW_IOX2_LOG_ERROR("the callback thread stopped: %s", ::iox2::bb::into<const char*>(result.error()));
        std::lock_guard<std::mutex> lock{m_mutex};
        m_stop = true;
        m_changes_applied.notify_all();
    }
}

auto CallbackThread::on_event(const AttachmentId& id) -> ::iox2::CallbackProgression {
    using ::iox2::CallbackProgression;

    std::lock_guard<std::mutex> lock{m_mutex};
    if (id == m_wake_id.value()) {
        (void)m_wake->drain();
        if (m_stop) {
            return CallbackProgression::Stop;
        }
        if (m_changed) {
            m_removed_guards.clear();
            attach_new_callbacks();
            m_changed = false;
            m_changes_applied.notify_all();
        }
        return CallbackProgression::Continue;
    }

    if (auto callback_id = m_ids.find(id); callback_id != m_ids.end()) {
        call(m_callbacks.at(callback_id->second));
    }
    return CallbackProgression::Continue;
}

void CallbackThread::attach_new_callbacks() {
    for (auto entry = m_callbacks.begin(); entry != m_callbacks.end();) {
        if (entry->second.guard.has_value() || attach(entry->first, entry->second)) {
            ++entry;
        } else {
            entry = m_callbacks.erase(entry);
        }
    }
}

auto CallbackThread::attach(uint64_t id, Callback& callback) -> bool {
    auto guard = m_waitset->attach_notification(callback.wake_up);
    if (!guard.has_value()) {
        RMW_IOX2_LOG_ERROR("failed to attach a callback: %s", ::iox2::bb::into<const char*>(guard.error()));
        return false;
    }
    m_ids.emplace(AttachmentId::from_guard(guard.value()), id);
    callback.guard.emplace(std::move(guard.value()));
    return true;
}

void CallbackThread::call(const Callback& callback) {
    if (auto count = callback.count_events(); count > 0) {
        callback.callback(callback.user_data, count);
    }
}

} // namespace rmw::iox2
