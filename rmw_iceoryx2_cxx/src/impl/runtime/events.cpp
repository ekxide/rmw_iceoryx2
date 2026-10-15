// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/impl/runtime/events.hpp"

#include "rmw_iceoryx2_cxx/impl/common/create.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

namespace rmw::iox2
{

auto Events::gid_of(const ::iox2::bb::Optional<::iox2::RawIdType>& raw_id) -> Gid {
    Gid gid{};
    if (raw_id.has_value()) {
        std::copy_n(raw_id.value().unchecked_access().data(), gid.size(), gid.begin());
    }
    return gid;
}

Events::Events(Iceoryx2& iox2, std::shared_ptr<CallbackThread> callback_thread, ListMatched list_matched)
    : m_iox2{iox2}
    , m_callback_thread{std::move(callback_thread)}
    , m_list_matched{std::move(list_matched)} {
}

auto Events::listen() -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    std::lock_guard<std::mutex> lock{m_mutex};
    if (m_graph_guard_condition.has_value()) {
        return {};
    }
    if (!create_in_place(m_graph_guard_condition, m_iox2.get()).has_value()) {
        RMW_IOX2_CHAIN_ERROR_MSG("failed to create the graph listener of the events");
        return err(ErrorType::GUARD_CONDITION_CREATION_FAILURE);
    }
    m_matched = m_list_matched();
    return {};
}

void Events::update_matched() {
    if (m_graph_guard_condition->drain()) {
        m_matched = m_list_matched();
    }
}

auto Events::take(rmw_event_type_t event_type, void* event_info) -> bool {
    std::lock_guard<std::mutex> lock{m_mutex};
    switch (event_type) {
    case RMW_EVENT_PUBLICATION_MATCHED:
    case RMW_EVENT_SUBSCRIPTION_MATCHED: {
        update_matched();
        auto& status = *static_cast<rmw_matched_status_t*>(event_info);
        const auto added = static_cast<size_t>(std::count_if(
            m_matched.begin(), m_matched.end(), [this](const Gid& gid) { return m_taken.count(gid) == 0; }));
        m_total_count += added;
        status.total_count = m_total_count;
        status.total_count_change = added;
        status.current_count = m_matched.size();
        status.current_count_change = static_cast<int32_t>(m_matched.size()) - static_cast<int32_t>(m_taken.size());
        m_taken = m_matched;
        return true;
    }
    default:
        return false;
    }
}

auto Events::is_ready(rmw_event_type_t event_type) -> bool {
    std::lock_guard<std::mutex> lock{m_mutex};
    switch (event_type) {
    case RMW_EVENT_PUBLICATION_MATCHED:
    case RMW_EVENT_SUBSCRIPTION_MATCHED:
        update_matched();
        return m_matched != m_taken;
    default:
        return false;
    }
}

auto Events::guard_condition() -> GuardCondition& {
    return m_graph_guard_condition.value();
}

auto Events::set_callback(rmw_event_type_t event_type, rmw_event_callback_t callback, const void* user_data)
    -> ::iox2::bb::Expected<void, ErrorType> {
    using ::iox2::bb::err;

    switch (event_type) {
    case RMW_EVENT_PUBLICATION_MATCHED:
    case RMW_EVENT_SUBSCRIPTION_MATCHED: {
        std::lock_guard<std::mutex> callback_lock{m_callback_mutex};
        m_callback.reset();
        if (callback == nullptr) {
            return {};
        }
        {
            std::lock_guard<std::mutex> lock{m_mutex};
            if (!m_callback_guard_condition.has_value()) {
                if (!create_in_place(m_callback_guard_condition, m_iox2.get()).has_value()) {
                    RMW_IOX2_CHAIN_ERROR_MSG("failed to create the graph listener of the event callback");
                    return err(ErrorType::GUARD_CONDITION_CREATION_FAILURE);
                }
                m_notified = m_taken;
            }
        }
        auto count_changes = [this]() -> uint64_t {
            std::lock_guard<std::mutex> lock{m_mutex};
            (void)m_callback_guard_condition->drain();
            auto matched = m_list_matched();
            std::set<Gid> changes;
            std::set_symmetric_difference(m_notified.begin(),
                                          m_notified.end(),
                                          matched.begin(),
                                          matched.end(),
                                          std::inserter(changes, changes.end()));
            m_notified = std::move(matched);
            return changes.size();
        };
        auto added =
            m_callback_thread->add(m_callback_guard_condition->file_descriptor(), count_changes, callback, user_data);
        if (!added.has_value()) {
            return err(ErrorType::CALLBACK_FAILURE);
        }
        m_callback.emplace(std::move(added.value()));
        return {};
    }
    default:
        return {};
    }
}

} // namespace rmw::iox2
