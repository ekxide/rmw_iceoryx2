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


/// An index used by the RMW to track entities attached to the waitset.
using RmwIndex = size_t;
enum class WaitableType { SUBSCRIPTION, GUARD_CONDITION };

class WaitSetImpl;

template <>
struct Error<WaitSetImpl>
{
    using Type = WaitSetError;
};

/// @brief Implementation of the RMW wait set for iceoryx2
/// @details Waitable entities are tracked in upper layers (i.e. RCL) using indices.
///          This implementation maps these indices to iceoryx2 listeners for events signifying
///          work is available related to the associated the entities.
class RMW_PUBLIC WaitSetImpl
{
    using Duration = ::iox::units::Duration;
    using AttachmentId = ::iox2::WaitSetAttachmentId<::iox2::ServiceType::Ipc>;
    using ServiceName = ::iox2::ServiceName;
    using Guard = ::iox2::WaitSetGuard<::iox2::ServiceType::Ipc>;
    using IceoryxWaitSet = ::iox2::WaitSet<::iox2::ServiceType::Ipc>;
    using IceoryxListener = ::iox2::Listener<::iox2::ServiceType::Ipc>;

    /// @brief Associates an iceoryx2 listener with its service name
    struct ListenerStorage
    {
        std::string service_name;
        IceoryxListener listener;
    };
    using StorageIndex = std::vector<ListenerStorage>::size_type;

    /// @brief Description of a waitable that has been staged
    struct StagedWaitable
    {
        StorageIndex storage_index;
        WaitableType waitable_type;
        RmwIndex rmw_index;
    };

    /// @brief Description of a waitable that has been triggered
    struct TriggeredWaitable
    {
        WaitableType waitable_type;
        RmwIndex rmw_index;
    };

    /// @brief Storage for waitset attachments containing the guard and attachment ID
    /// @details Manages the lifetime of a waitset attachment and provides access to its ID and associated waitable
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
    /// @brief Constructor for the WaitSetImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context to which this waitset is bound to. Must outlive the WaitSetImpl instance.
    WaitSetImpl(CreationLock, iox::optional<ErrorType>& error, ContextImpl& context);

    /// @brief Attach a guard condition to the waitset
    /// @param[in] rmw_index The index used to track the guard condition in the upper layers
    /// @param[in] guard_condition The guard condition to attach to the waitset
    auto attach(RmwIndex rmw_index, GuardConditionImpl& guard_condition) -> iox::expected<void, ErrorType>;

    /// @brief Attach a subscriber to the waitset
    /// @param[in] rmw_index The index used to track the subscriber in the upper layers
    /// @param[in] guard_condition The subscriber to attach to the waitset
    auto attach(RmwIndex rmw_index, SubscriberImpl& subscriber) -> iox::expected<void, ErrorType>;

    /// @brief Detach all entities from the waitset
    /// @note Detached entities will not be waited on when calling wait()
    auto detach_all() -> void;

    // TODO: Return multiple triggered waitables
    /// @brief Block the thread until at least one attached entity is triggered or the timeout is reached
    /// @param timeout Optional timeout after which waiting is stopped. If null waits indefinitely. If 0 does not wait
    ///                at all.
    /// @returns The triggered waitable
    auto wait(const iox::optional<Duration>& timeout = iox::nullopt)
        -> iox::expected<iox::optional<TriggeredWaitable>, ErrorType>;

private:
    /// @brief Creates a listener for the given service, if not already created.
    /// @note If the listener was previously already created, skips creation and reuses the existing listener.
    /// @param[in] The service name to use for the iceoryx2 listener
    auto create_listener(const std::string& service_name) -> iox::expected<StorageIndex, ErrorType>;

    /// @brief Stages a listener to be waited on.
    /// @param[in] entity_type The entity type that the listener is being notified for
    /// @param[in] storage_index The index that the associated listner is stored in the ListenerStorage
    /// @param[in] rmw_index The index for the associated entity tracked by the upper layers
    auto stage_listener(WaitableType entity_type, StorageIndex storage_index, RmwIndex rmw_index) -> void;


    /// @brief Get the listener at the given storage index.
    /// @param[in] storage_index The index to retrieve the listener from
    /// @return The stored listener found at the given index, or nullopt if index is invalid
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
