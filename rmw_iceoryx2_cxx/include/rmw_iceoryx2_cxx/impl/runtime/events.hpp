// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_EVENTS_HPP_
#define RMW_IOX2_RUNTIME_EVENTS_HPP_

#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/event.h"
#include "rmw/event_callback_type.h"
#include "rmw/events_statuses/matched.h"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/callback_thread.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>

namespace rmw::iox2
{

/// @brief The events of a publisher or a subscription, owned by it
class RMW_PUBLIC Events
{
public:
    using ErrorType = EventsError;
    using Gid = std::array<uint8_t, ::iox2::UNIQUE_PORT_ID_LENGTH>;
    using ListMatched = std::function<std::set<Gid>()>;

    /// @brief Converts the unique id of an iceoryx2 port
    /// @param[in] raw_id The unique id of the port
    /// @return The gid, zero if the port has no id
    static auto gid_of(const ::iox2::bb::Optional<::iox2::RawIdType>& raw_id) -> Gid;

    /// @brief Creates the events of a publisher or a subscription, which listen to the graph after listen()
    /// @param[in] iox2 The iceoryx2 node of the publisher or subscription
    /// @param[in] callback_thread The thread that calls the callbacks of the events
    /// @param[in] list_matched Lists the endpoints matched with the publisher or subscription
    Events(Iceoryx2& iox2, std::shared_ptr<CallbackThread> callback_thread, ListMatched list_matched);
    Events(const Events&) = delete;
    Events(Events&&) = delete;
    auto operator=(const Events&) -> Events& = delete;
    auto operator=(Events&&) -> Events& = delete;

    /// @brief Creates the graph listener of the events, if it does not exist yet
    /// @return Expected containing void if successful, error otherwise
    auto listen() -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Takes the status of an event type
    /// @param[in] event_type The event type
    /// @param[out] event_info The status of the event type
    /// @return True if the event type has a status
    auto take(rmw_event_type_t event_type, void* event_info) -> bool;

    /// @brief Check if the status of an event type changed since the last take
    /// @param[in] event_type The event type
    /// @return True if it changed
    auto is_ready(rmw_event_type_t event_type) -> bool;

    /// @brief Get the guard condition that is triggered by changes of the graph
    /// @return Reference to the guard condition
    auto guard_condition() -> GuardCondition&;

    /// @brief Sets the callback that is called with the number of changes of the status of an event type
    /// @details A new callback is called right away with the changes since the last take
    /// @param[in] event_type The event type
    /// @param[in] callback The callback, nullptr removes the callback
    /// @param[in] user_data Passed to the callback
    /// @return Expected containing void if successful, error otherwise
    auto set_callback(rmw_event_type_t event_type, rmw_event_callback_t callback, const void* user_data)
        -> ::iox2::bb::Expected<void, ErrorType>;

private:
    /// @brief Lists the matched endpoints again if the graph changed
    void update_matched();

    std::reference_wrapper<Iceoryx2> m_iox2;
    std::shared_ptr<CallbackThread> m_callback_thread;
    ListMatched m_list_matched;
    std::mutex m_mutex;
    ::iox2::bb::Optional<GraphGuardCondition> m_graph_guard_condition;
    ::iox2::bb::Optional<GraphGuardCondition> m_callback_guard_condition;
    std::set<Gid> m_matched;
    std::set<Gid> m_taken;
    uint64_t m_total_count{0};
    std::set<Gid> m_notified;
    ::iox2::bb::Optional<CallbackThread::Registration> m_callback;
};

} // namespace rmw::iox2

#endif
