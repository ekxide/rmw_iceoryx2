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
#include "iox2/file_descriptor.hpp"
#include "rmw/event_callback_type.h"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace rmw::iox2
{

/// @brief Calls the callbacks of the entities of a context from one thread
class RMW_PUBLIC CallbackThread : public std::enable_shared_from_this<CallbackThread>
{
public:
    using ErrorType = CallbackThreadError;
    using Listener = Iceoryx2::InterProcess::Listener;
    using CountEvents = std::function<uint64_t()>;
    using OnWakeUp = std::function<void()>;
    using CallbackId = uint64_t;

    /// @brief A callback of the thread, removed when the registration is destroyed
    class RMW_PUBLIC Registration
    {
    public:
        Registration(const Registration&) = delete;
        Registration(Registration&& other) noexcept;
        auto operator=(const Registration&) -> Registration& = delete;
        auto operator=(Registration&& other) noexcept -> Registration&;
        /// @brief Removes the callback, it is neither running nor called after this returns
        ~Registration();

    private:
        friend class CallbackThread;
        explicit Registration(std::shared_ptr<CallbackThread> thread);

        static inline std::atomic<CallbackId> s_next_id{0};
        std::shared_ptr<CallbackThread> m_thread;
        CallbackId m_id;
    };

    CallbackThread() = default;
    CallbackThread(const CallbackThread&) = delete;
    CallbackThread(CallbackThread&&) = delete;
    auto operator=(const CallbackThread&) -> CallbackThread& = delete;
    auto operator=(CallbackThread&&) -> CallbackThread& = delete;
    ~CallbackThread();

    /// @brief Starts the thread, which then waits for callbacks to call
    /// @return Expected containing void if successful, error otherwise
    auto start() -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Adds a callback that is called with the number of notifications the listener receives
    /// @details The callback is called right away with the notifications the listener already received
    /// @param[in] listener The listener of the entity, which must outlive the registration
    /// @param[in] callback The callback
    /// @param[in] user_data Passed to the callback
    /// @return Expected containing the registration of the callback if successful, error otherwise
    auto add(Listener& listener, rmw_event_callback_t callback, const void* user_data)
        -> ::iox2::bb::Expected<Registration, ErrorType>;

    /// @brief Adds a callback that is called with the number of events count_events returns whenever the file
    ///        descriptor is readable
    /// @details The callback is called right away with the events that already happened
    /// @param[in] file_descriptor The file descriptor that is readable when events might have happened
    /// @param[in] count_events Returns the number of events since its last call
    /// @param[in] callback The callback
    /// @param[in] user_data Passed to the callback
    /// @return Expected containing the registration of the callback if successful, error otherwise
    auto add(::iox2::FileDescriptorView file_descriptor,
             CountEvents count_events,
             rmw_event_callback_t callback,
             const void* user_data) -> ::iox2::bb::Expected<Registration, ErrorType>;

    /// @brief Adds a function that is called whenever the file descriptor is readable
    /// @details The function is called right away once, for what happened before it was added
    /// @param[in] file_descriptor The file descriptor that is readable when the function has something to do
    /// @param[in] on_wake_up The function
    /// @return Expected containing the registration of the function if successful, error otherwise
    auto add(::iox2::FileDescriptorView file_descriptor, OnWakeUp on_wake_up)
        -> ::iox2::bb::Expected<Registration, ErrorType>;

private:
    using Guard = Iceoryx2::WaitSet::Guard;
    using AttachmentId = Iceoryx2::WaitSet::AttachmentId;

    struct Callback
    {
        ::iox2::FileDescriptorView file_descriptor;
        OnWakeUp on_wake_up;
        ::iox2::bb::Optional<Guard> guard;
    };

    void remove(CallbackId id);
    void synchronize(std::unique_lock<std::mutex>& lock);
    void run();
    auto on_event(const AttachmentId& id) -> ::iox2::CallbackProgression;
    void attach_new_callbacks();
    auto attach(Callback& callback) -> bool;
    static void call(const Callback& callback);

    std::mutex m_mutex;
    std::condition_variable m_changes_applied;
    bool m_changed{false};
    bool m_stop{false};
    ::iox2::bb::Optional<UserGuardCondition> m_wake;
    ::iox2::bb::Optional<Iceoryx2::WaitSet::Handle> m_waitset;
    ::iox2::bb::Optional<Guard> m_wake_guard;
    ::iox2::bb::Optional<AttachmentId> m_wake_id;
    std::map<CallbackId, Callback> m_callbacks;
    std::vector<Guard> m_removed_guards;
    std::thread m_thread;
};

} // namespace rmw::iox2

#endif
