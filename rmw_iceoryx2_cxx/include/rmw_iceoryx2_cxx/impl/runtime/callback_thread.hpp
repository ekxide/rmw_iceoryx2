// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_CALLBACK_THREAD_HPP_
#define RMW_IOX2_RUNTIME_CALLBACK_THREAD_HPP_

#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/callback_progression.hpp"
#include "rmw/event_callback_type.h"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"

#include <condition_variable>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

namespace rmw::iox2
{

/// @brief Calls the new data callbacks of the entities of a context from one thread, started by the first callback
class RMW_PUBLIC CallbackThread
{
public:
    using ErrorType = CallbackThreadError;
    using Listener = Iceoryx2::InterProcess::Listener;

    CallbackThread() = default;
    CallbackThread(const CallbackThread&) = delete;
    CallbackThread(CallbackThread&&) = delete;
    auto operator=(const CallbackThread&) -> CallbackThread& = delete;
    auto operator=(CallbackThread&&) -> CallbackThread& = delete;
    ~CallbackThread();

    /// @brief Calls the callback with the number of notifications the listener receives, first with those it
    ///        already received
    /// @param[in] listener The listener of the entity
    /// @param[in] callback The callback, nullptr removes the callback of the listener
    /// @param[in] user_data Passed to the callback
    /// @return Expected containing void if successful, error otherwise
    auto set_callback(Listener& listener, rmw_event_callback_t callback, const void* user_data)
        -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Removes the callback of the listener, it is neither running nor called after this returns
    /// @param[in] listener The listener of the entity
    void remove_callback(Listener& listener);

private:
    using Guard = Iceoryx2::WaitSet::Guard;
    using AttachmentId = Iceoryx2::WaitSet::AttachmentId;

    struct Callback
    {
        rmw_event_callback_t callback;
        const void* user_data;
        ::iox2::bb::Optional<Guard> guard;
    };

    auto start() -> ::iox2::bb::Expected<void, ErrorType>;
    void apply_changes(std::unique_lock<std::mutex>& lock);
    void run();
    auto on_event(const AttachmentId& id) -> ::iox2::CallbackProgression;
    void attach_new_callbacks();
    auto attach(Listener* listener, Callback& callback) -> bool;
    static void call(Listener& listener, const Callback& callback);

    std::mutex m_mutex;
    std::condition_variable m_changes_applied;
    bool m_changed{false};
    bool m_stop{false};
    ::iox2::bb::Optional<UserGuardCondition> m_wake;
    ::iox2::bb::Optional<Iceoryx2::WaitSet::Handle> m_waitset;
    ::iox2::bb::Optional<Guard> m_wake_guard;
    ::iox2::bb::Optional<AttachmentId> m_wake_id;
    std::map<Listener*, Callback> m_callbacks;
    std::map<AttachmentId, Listener*> m_listeners;
    std::vector<Guard> m_removed_guards;
    std::thread m_thread;
};

} // namespace rmw::iox2

#endif
