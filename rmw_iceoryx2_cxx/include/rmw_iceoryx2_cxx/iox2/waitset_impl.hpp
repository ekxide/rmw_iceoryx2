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
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

#include <vector>

namespace rmw::iox2
{

// TODO: Move somewhere else
template <typename T>
static constexpr bool always_false = false;

enum class AttachmentType { SUBSCRIBER, GUARD_CONDITION };

/// An index used by the RMW to track entities attached to the waitset.
using RmwIndex = size_t;
enum class WaitableType { SUBSCRIBER, GUARD_CONDITION };

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
    using ServiceName = ::iox2::ServiceName;
    using WaitSet = Iceoryx2::WaitSet::Handle;
    using WaitSetGuard = Iceoryx2::WaitSet::Guard;
    using WaitSetAttachmentId = Iceoryx2::WaitSet::AttachmentId;
    using GuardConditionListener = Iceoryx2::Local::Listener;
    using SubscriberListener = Iceoryx2::InterProcess::Listener;

    /// @brief Associates an iceoryx2 listener with its service name
    template <typename ListenerType>
    struct ListenerDetails
    {
        std::string service_name;
        ListenerType listener;
    };
    using StorageIndex = size_t;

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
    class AttachmentDetails
    {
    public:
        AttachmentDetails(WaitSetGuard&& guard)
            : m_guard{std::move(guard)}
            , m_id{WaitSetAttachmentId::from_guard(m_guard)} {
        }

        AttachmentDetails(WaitSetGuard&& guard, const StagedWaitable& staged_waitable)
            : m_guard{std::move(guard)}
            , m_id{WaitSetAttachmentId::from_guard(m_guard)}
            , m_waitable{staged_waitable} {
        }

        auto id() -> WaitSetAttachmentId& {
            return m_id;
        }

        auto waitable() -> iox::optional<StagedWaitable>& {
            return m_waitable;
        }

    private:
        WaitSetGuard m_guard;
        WaitSetAttachmentId m_id;
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
    template <typename ListenerType>
    auto create_listener(const std::string& service_name) -> iox::expected<StorageIndex, ErrorType>;

    /// @brief Stages a listener to be waited on.
    /// @param[in] entity_type The entity type that the listener is being notified for
    /// @param[in] storage_index The index that the associated listner is stored in the ListenerStorage
    /// @param[in] rmw_index The index for the associated entity tracked by the upper layers
    auto stage_listener(WaitableType entity_type, StorageIndex storage_index, RmwIndex rmw_index) -> void;

    /// @brief Get the listener at the given storage index.
    /// @param[in] storage_index The index to retrieve the listener from
    /// @return The stored listener found at the given index, or nullopt if index is invalid
    template <typename ListenerType>
    inline auto get_stored_listener(StorageIndex storage_index) -> iox::optional<ListenerDetails<ListenerType>*>;

    template <typename ListenerType>
    inline auto listener_storage() -> std::vector<ListenerDetails<ListenerType>>&;

    // TODO: rename
    template <typename ListenerType>
    inline static constexpr auto service_type = []() -> ::iox2::ServiceType {
        if constexpr (std::is_same_v<ListenerType, GuardConditionListener>) {
            return ::iox2::ServiceType::Local;
        } else if constexpr (std::is_same_v<ListenerType, SubscriberListener>) {
            return ::iox2::ServiceType::Ipc;
        } else {
            static_assert(std::false_type::value, "Unsupported listener type");
        }
    }();

private:
    ContextImpl& m_context;
    iox::optional<WaitSet> m_waitset;

    // Storage for all attached listeners.
    // Listeners for entities are created on first attachment, and re-used in subsequent calls.
    // WARNING: Listeners must not be removed once added to the storage as this invalidates held storage indicies.
    // TODO: A less error-prone solution. This is the quickest "dumb" implementation to get things working.
    std::vector<ListenerDetails<GuardConditionListener>> m_guard_condition_listeners;
    std::vector<ListenerDetails<SubscriberListener>> m_subscriber_listeners;

    // Listeners staged to be waited on in the next wait call.
    // Maps the attachment to the index used in the RMW for tracking.
    std::vector<StagedWaitable> m_staged_waitables;
};

// ===================================================================================================================

template <typename ListenerType>
auto WaitSetImpl::create_listener(const std::string& service_name) -> iox::expected<StorageIndex, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    auto& storage = listener_storage<ListenerType>();

    auto it = std::find_if(storage.begin(), storage.end(), [&service_name](const auto& listener) {
        return listener.service_name == service_name;
    });

    if (it == storage.end()) {
        auto service_result =
            m_context.iox2().service_builder<service_type<ListenerType>>(service_name).event().open_or_create();
        if (service_result.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(service_result.error()));
            return err(ErrorType::SERVICE_CREATION_FAILURE);
        }
        auto& service = service_result.value();

        auto listener = service.listener_builder().create();
        if (listener.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(listener.error()));
            return err(ErrorType::LISTENER_CREATION_FAILURE);
        }

        storage.emplace_back(ListenerDetails<ListenerType>{service_name, std::move(listener.value())});
        auto storage_index = static_cast<StorageIndex>(storage.size() - 1);
        return ok(storage_index);
    } else {
        // An iceoryx2 listener already exists. Reuse it and mark it for attachment.
        auto storage_index = static_cast<StorageIndex>(it - storage.begin());
        return ok(storage_index);
    }
}

template <typename ListenerType>
auto WaitSetImpl::get_stored_listener(StorageIndex storage_index) -> iox::optional<ListenerDetails<ListenerType>*> {
    auto& storage = listener_storage<ListenerType>();

    if (storage_index < storage.size()) {
        return &storage[storage_index];
    }
    return iox::nullopt;
}

template <typename ListenerType>
inline auto WaitSetImpl::listener_storage() -> std::vector<ListenerDetails<ListenerType>>& {
    if constexpr (std::is_same_v<ListenerType, GuardConditionListener>) {
        return m_guard_condition_listeners;
    } else if constexpr (std::is_same_v<ListenerType, SubscriberListener>) {
        return m_subscriber_listeners;
    } else {
        // TODO: Panic
    }
}

} // namespace rmw::iox2

#endif
