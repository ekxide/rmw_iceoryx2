// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"

#include "iox2/event_id.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

namespace rmw::iox2
{

GuardConditionImpl::GuardConditionImpl(uint32_t id, Notifier&& notifier)
    : m_id{id}
    , m_notifier{std::move(notifier)} {};

uint32_t GuardConditionImpl::id() const {
    return m_id;
}

bool GuardConditionImpl::trigger(const iox::optional<size_t>& id) {
    using ::iox2::EventId;

    bool result = true;
    if (id.has_value()) {
        if (m_notifier.notify_with_custom_event_id(EventId(id.value())).has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to send notification");
            result = false;
        };
    } else {
        if (m_notifier.notify().has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG("failed to send notification");
            result = false;
        }
    }

    return result;
}

} // namespace rmw::iox2
