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
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

#include <vector>

namespace rmw::iox2
{

enum class AttachmentType { SUBSCRIPTION, GUARD_CONDITION };

/*
 * An index used by the RMW to track entities attached to the waitset.
 */
using RmwIndex = size_t;
enum class WaitableType { SUBSCRIPTION, GUARD_CONDITION };

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

    struct ListenerStorage
    {
        std::string service_name;
        IceoryxListener listener;
    };
    using StorageIndex = std::vector<ListenerStorage>::size_type;

    struct StagedWaitable
    {
        StorageIndex storage_index;
        WaitableType waitable_type;
        RmwIndex rmw_index;
    };
    struct TriggeredWaitable
    {
        WaitableType waitable_type;
        RmwIndex rmw_index;
    };

    class WaitSetAttachmentStorage
    {
    public:
        WaitSetAttachmentStorage(Guard&& guard)
            : m_guard{std::move(guard)}
            , m_id{AttachmentId::from_guard(m_guard)} {
        }

        WaitSetAttachmentStorage(Guard&& guard, const StagedWaitable& staged_waitable)
            : m_guard{std::move(guard)}
            , m_id{AttachmentId::from_guard(m_guard)}
            , m_waitable{staged_waitable} {
        }

        auto id() -> AttachmentId& {
            return m_id;
        }

        auto waitable() -> iox::optional<StagedWaitable>& {
            return m_waitable;
        }

    private:
        Guard m_guard;
        AttachmentId m_id;
        iox::optional<StagedWaitable> m_waitable;
    };

public:
    using ErrorType = Error<WaitSetImpl>::Type;

public:
    /**
     * @brief The iceoryx2 implementation of a WaitSet.
     *
     * @param context The context to which this waitset is bound to. Must outlive the WaitSetImpl instance.
     */
    WaitSetImpl(CreationLock, iox::optional<ErrorType>& error, ContextImpl& context);

    auto attach(RmwIndex rmw_index, GuardConditionImpl& guard_condition) -> iox::expected<void, ErrorType>;
    auto attach(RmwIndex rmw_index, SubscriberImpl& subscriber) -> iox::expected<void, ErrorType>;
    auto detach_all() -> void;

    auto wait(const iox::optional<Duration>& timeout = iox::nullopt)
        -> iox::expected<iox::optional<TriggeredWaitable>, ErrorType>;

private:
    /*
     * @brief Creates a listener for the given service, if not already created.
     *
     * If the listener was previously already created, skips creation and reuses the existing listener.
     */
    auto create_listener(const std::string& service_name) -> iox::expected<StorageIndex, ErrorType>;

    /*
     * @brief Stages a listener to be waited on.
     */
    auto stage_listener(WaitableType entity_type, StorageIndex storage_index, RmwIndex rmw_index) -> void;

    /*
     * Get the listener at the given storage index.
     */
    auto get_listener(StorageIndex storage_index) -> iox::optional<ListenerStorage*>;

private:
    ContextImpl& m_context;
    iox::optional<IceoryxWaitSet> m_waitset;

    // Storage for all attached listeners.
    // Listeners for entities are created on first attachment, and re-used in subsequent calls.
    // WARNING: Listeners must not be removed once added to the storage as this invalidates held storage indicies.
    // TODO: A less error-prone solution. This is the quickest "dumb" implementation to get things working.
    std::vector<ListenerStorage> m_listener_storage;

    // Listeners staged to be waited on in the next wait call.
    // Maps the attachment to the index used in the RMW for tracking.
    std::vector<StagedWaitable> m_staged_waitables;
};

} // namespace rmw::iox2

#endif
