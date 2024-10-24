// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_WAITSET_IMPL_HPP_
#define RMW_IOX2_WAITSET_IMPL_HPP_

#include "iox/duration.hpp"
#include "iox/optional.hpp"
#include "iox2/listener.hpp"
#include "iox2/waitset.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

namespace rmw::iox2
{

class WaitSetImpl;

template <>
struct Error<WaitSetImpl>
{
    using Type = WaitSetError;
};

class RMW_PUBLIC WaitSetImpl
{
    using Duration = ::iox::units::Duration;
    using AttachmentId = ::iox2::WaitSetAttachmentId<::iox2::ServiceType::Ipc>;
    using ServiceName = ::iox2::ServiceName;
    using Guard = ::iox2::WaitSetGuard<::iox2::ServiceType::Ipc>;
    using IceoryxWaitSet = ::iox2::WaitSet<::iox2::ServiceType::Ipc>;
    using IceoryxListener = ::iox2::Listener<::iox2::ServiceType::Ipc>;
    // TODO: Remove use of std. Current solution is only for prototyping.
    using Callback = std::function<void(void)>;

public:
    using ErrorType = Error<WaitSetImpl>::Type;

public:
    /**
     * @brief The iceoryx2 implementation of a WaitSet.
     *
     * @param context The context to which this waitset is bound to. Must outlive the WaitSetImpl instance.
     */
    WaitSetImpl(iox::optional<ErrorType>& error, ContextImpl& context);

    // TODO: Don't pass callback by value
    auto attach(GuardConditionImpl& guard_condition) -> iox::expected<void, ErrorType>;
    auto attach(SubscriberImpl& subscriber) -> iox::expected<void, ErrorType>;
    auto wait(const Duration& timeout = iox::units::Duration::fromSeconds(0)) -> iox::expected<void, ErrorType>;

private:
    auto attach_listener(const std::string& service_name) -> iox::expected<void, ErrorType>;
    auto on_trigger(AttachmentId& id) -> void;

private:
    ContextImpl& m_context;
    iox::optional<IceoryxWaitSet> m_waitset;

    // TODO: Remove use of std. Current solution is only for prototyping.
    // TODO: String comparison of service name is expensive. Change it.
    struct AttachedListener
    {
        std::string service_name;
        AttachmentId id;
        Guard guard;
        IceoryxListener listener;
    };
    std::vector<AttachedListener> m_attached_listeners;
};

} // namespace rmw::iox2

#endif
