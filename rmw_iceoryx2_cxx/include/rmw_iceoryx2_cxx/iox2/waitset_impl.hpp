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
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rmw_iceoryx2_cxx/iox2/context_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"

#include <vector>

namespace rmw::iox2
{

// TODO: Move somewhere else
template <typename T>
static constexpr bool always_false = false;

/// An index used by the RMW to track entities attached to the waitset.
using RmwIndex = size_t;

/// Types of entities that the waitset is capable of waiting on
enum class WaitableEntity { SUBSCRIBER, GUARD_CONDITION };

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

    /// @brief Helper to map listener types to their corresponding iceoryx2 service type
    /// @tparam ListenerType The type of listener to get the service type for
    /// @return The iceoryx2 service type corresponding to the listener type
    template <typename ListenerType>
    inline static constexpr auto ServiceType = []() -> Iceoryx2::ServiceType {
        if constexpr (std::is_same_v<ListenerType, GuardConditionListener>) {
            return Iceoryx2::ServiceType::Local;
        } else if constexpr (std::is_same_v<ListenerType, SubscriberListener>) {
            return Iceoryx2::ServiceType::Ipc;
        } else {
            static_assert(std::false_type::value, "Unsupported listener type");
        }
    }();

    /// @brief Details about a listener including its service name and the listener itself
    /// @tparam ListenerType The type of listener (GuardConditionListener or SubscriberListener)
    template <typename ListenerType>
    struct ListenerDetails
    {
        std::string service_name;
        ListenerType listener;
    };
    using StorageIndex = size_t;

    /// @brief Mapping from RMW index to a stored listener.
    /// @details Allows for triggered listeners to be mapped back to the index that RMW uses for tracking
    struct RmwMapping
    {
        WaitableEntity waitable_type;
        StorageIndex storage_index;
        RmwIndex rmw_index;
    };

    /// @brief Description of a waitable that has been triggered
    struct TriggeredWaitable
    {
        TriggeredWaitable(const RmwMapping& mapping)
            : waitable_type{mapping.waitable_type}
            , rmw_index{mapping.rmw_index} {
        }
        WaitableEntity waitable_type;
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

        AttachmentDetails(WaitSetGuard&& guard, const RmwMapping& staged_waitable)
            : m_guard{std::move(guard)}
            , m_id{WaitSetAttachmentId::from_guard(m_guard)}
            , m_rmw_mapping{staged_waitable} {
        }

        auto id() const -> const WaitSetAttachmentId& {
            return m_id;
        }

        auto mapping() const -> const RmwMapping& {
            return m_rmw_mapping;
        }

    private:
        WaitSetGuard m_guard;
        WaitSetAttachmentId m_id;
        RmwMapping m_rmw_mapping;
    };

    /// @brief Context for individual wait calls
    /// @details An instance of this is created for each wait call to track all attachments.
    struct WaitContext
    {
        iox::optional<AttachmentDetails> attached_timeout;
        std::vector<AttachmentDetails> attached_listeners;
        iox::optional<TriggeredWaitable> result;
    };

public:
    using ErrorType = Error<WaitSetImpl>::Type;

public:
    /// @brief Constructor for the WaitSetImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context to which this waitset is bound to. Must outlive the WaitSetImpl instance.
    WaitSetImpl(CreationLock, iox::optional<ErrorType>& error, ContextImpl& context);

    /// @brief Maps a guard condition to an RMW index
    /// @details A listener is created for the mapped guard condition which will be waited on in subsequent wait calls
    ///          unless unmapped
    /// @param[in] rmw_index The index used to track the guard condition in the RMW
    /// @param[in] guard_condition The guard condition to be mapped
    auto map(RmwIndex rmw_index, GuardConditionImpl& guard_condition) -> iox::expected<void, ErrorType>;

    /// @brief Maps a subscriber to an RMW index
    /// @details A listener is created for the mapped subscriber which will be waited on in subsequent wait calls
    ///          unless unmapped
    /// @param[in] rmw_index The index used to track the subscriber in the RMW
    /// @param[in] subscribe The subscriber to be mapped
    auto map(RmwIndex rmw_index, SubscriberImpl& subscriber) -> iox::expected<void, ErrorType>;

    /// @brief Unmap all currently mapped waitable entities
    /// @note Unmapped entities will not be waited on in subsequent wait calls
    auto unmap_all() -> void;

    // NOTE: Each wait is creating a new WaitContext and (re)-attaching waitable entities.
    //       This is inefficient, but to improve it a more efficient way to compare waitable entities (other than
    //       service name string comparison) is required.
    //       With this, a WaitContext could be used and thus instead of detaching all attachments and reattching them in
    //       subsequent waits, a diff of currently mapped entities and entities already in the context can be done and
    //       processed accordingly.
    //       ... Maybe XOR of the unique IDs? Something to try when time permits.
    //
    /// @brief Block the thread until at least one attached entity is triggered or the timeout is reached.
    /// @details Although multiple events may be present on wake-up, only one is returned to the caller. Remaining
    ///          events can be consumed via subsequent wait calls.
    /// @param timeout Optional timeout after which waiting is stopped. If null waits indefinitely. If 0 does not wait
    ///                at all.
    /// @returns The triggered waitable
    auto wait(const iox::optional<Duration>& timeout = iox::nullopt)
        -> iox::expected<iox::optional<TriggeredWaitable>, ErrorType>;

private:
    /// @brief Gets the storage index of the listener for the provided service.
    /// @details Creates a listener for the service if one does not exist in the storage.
    /// @tparam ListenerType The type of listener (GuardConditionListener or SubscriberListener)
    /// @param[in] The service name to use for the iceoryx2 listener
    /// @return The index where the listener is stored in its specific storage
    template <typename ListenerType>
    auto get_storage_index(const std::string& service_name) -> iox::expected<StorageIndex, ErrorType>;

    /// @brief Get the listener at the given storage index.
    /// @tparam ListenerType The type of listener (GuardConditionListener or SubscriberListener)
    /// @param[in] storage_index The index to retrieve the listener from
    /// @details Listeners of different types are stored in different storages.
    ///
    /// This method retrieves the listener from the appropriate storage identified by the template argument.
    ///
    /// @note The storage index is unique within each storage type and is used to identify listeners of the same type
    /// @return Optional pointer to the listener details if found, nullopt otherwise
    template <typename ListenerType>
    inline auto get_stored_listener(StorageIndex storage_index) -> iox::optional<ListenerDetails<ListenerType>*>;

    /// @brief Maps a stored listener to an RMW index.
    /// @param[in] entity_type
    /// @param[in] storage_index The index where associated listener is stored in its given storage
    /// @param[in] rmw_index The index used by RMW to track attached entities.
    auto map_stored_listener(WaitableEntity entity_type, StorageIndex storage_index, RmwIndex rmw_index) -> void;

    /// @brief Get the storage of a specific listener type.
    /// @details Each listener type has its own dedicated storage. This method returns a reference to the
    ///          corresponding storage based on the template parameter.
    /// @tparam ListenerType The type of listener to get storage for (GuardConditionListener or SubscriberListener)
    /// @return Reference to the storage vector containing listeners of the specified type
    template <typename ListenerType>
    inline auto listener_storage() -> std::vector<ListenerDetails<ListenerType>>&;

    /// @brief Determine whether the waitset has a non-zero timeout.
    /// @return True if the waitset should time out.
    auto non_zero_timeout(const iox::optional<Duration>& timeout) const -> bool;

    /// @brief Determine if the waitset should block and wait to be notified according to provided timeout.
    /// @return True if the timeout is a value that indicates a wait is necessary, otherwise false
    auto should_block(const iox::optional<Duration>& timeout) const -> bool;

    /// @brief Attach a timeout to the waitset.
    /// @details Creates an interval attachment to the waitset that will trigger after the specified duration
    /// @param[in] timeout The duration after which the timeout should trigger
    /// @param[out] ctx The context for the given wait call where the timeout attachment will be stored
    /// @return Success if the timeout was attached, error otherwise
    auto attach_timeout(const Duration& timeout, WaitContext& ctx) -> ::iox::expected<void, ErrorType>;

    /// @brief Attach all mapped listeners to the waitset.
    /// @details For each mapped listener, creates a notification attachment to the waitset and stores the details.
    ///          If any attachment fails, returns an error immediately. Waiting should not proceed in this case.
    /// @param[out] ctx The context for the given wait call where the notification attachment will be stored
    /// @return Success if all listeners were attached, error otherwise
    auto attach_mapped_listeners(WaitContext& ctx) -> iox::expected<void, ErrorType>;

    /// @brief Attach a mapped listener to the waitset.
    /// @details Creates a notification attachment to the waitset for the given mapping. The mapping must reference
    ///          a valid listener in storage.
    /// @param[in] mapping The mapping containing details about the listener to attach
    /// @return Success with the attachment details if successful, error otherwise
    auto attach_mapped_listener(const RmwMapping& mapping) -> iox::expected<AttachmentDetails, ErrorType>;

    /// @brief Attach a mapped listener of a specific type to the waitset.
    /// @details Creates a notification attachment to the waitset for the given mapping. The mapping must reference
    ///          a valid listener in storage.
    /// @tparam ListenerType The type of listener (GuardConditionListener or SubscriberListener)
    /// @param[in] mapping The mapping containing details about the listener to attach
    /// @return Success with the attachment details if successful, error otherwise
    template <typename ListenerType>
    auto attach_mapped_listener_impl(const RmwMapping& mapping) -> iox::expected<AttachmentDetails, ErrorType>;

    /// @brief Process a triggered waitable entity
    /// @details Processes a triggered waitable entity by consuming the events from the associated listener
    /// @param[in] waitable_type The type of waitable entity that was triggered
    /// @param[in] storage_index The index where the triggered entity's listener is stored
    /// @return Success if all events were consumed successfully, error otherwise
    auto process_trigger(const WaitableEntity waitable_type,
                         const StorageIndex storage_index) -> iox::expected<void, ErrorType>;

private:
    ContextImpl& m_context;
    iox::optional<WaitSet> m_waitset;

    // Storage for all attached listeners.
    // Listeners for entities are created on first mapping, and re-used in subsequent calls.
    // WARNING: Listeners must not be removed once added to the storage as this invalidates held storage indicies.
    // TODO: A less error-prone solution. This is the quickest "dumb" implementation to get things working.
    std::vector<ListenerDetails<GuardConditionListener>> m_guard_condition_listeners;
    std::vector<ListenerDetails<SubscriberListener>> m_subscriber_listeners;

    // Listeners staged to be waited on in the next wait call.
    // Maps the attachment to the index used in the RMW for tracking.
    std::vector<RmwMapping> m_mapped_listeners;
};

// ===================================================================================================================

template <typename ListenerType>
auto WaitSetImpl::get_storage_index(const std::string& service_name) -> iox::expected<StorageIndex, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    auto& storage = listener_storage<ListenerType>();

    auto it = std::find_if(storage.begin(), storage.end(), [&service_name](const auto& listener) {
        return listener.service_name == service_name;
    });

    if (it == storage.end()) {
        auto service_result =
            m_context.iox2().service_builder<ServiceType<ListenerType>>(service_name).event().open_or_create();
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
        static_assert(always_false<ListenerType>, "Attempted to retrieve a listener of unknown type");
    }
}

template <typename ListenerType>
auto WaitSetImpl::attach_mapped_listener_impl(const RmwMapping& mapping)
    -> iox::expected<AttachmentDetails, ErrorType> {
    using ::iox::err;
    using ::iox::ok;

    if (auto result = get_stored_listener<ListenerType>(mapping.storage_index); result.has_value()) {
        auto& listener_details = result.value();
        auto guard = m_waitset->attach_notification(listener_details->listener.file_descriptor());
        if (guard.has_error()) {
            RMW_IOX2_CHAIN_ERROR_MSG(::iox::into<const char*>(guard.error()));
            return err(ErrorType::ATTACHMENT_FAILURE);
        }
        return ok(AttachmentDetails(std::move(guard.value()), mapping));
    }
    RMW_IOX2_CHAIN_ERROR_MSG("mapped listener not found in listener storage");
    return err(ErrorType::INVALID_STORAGE_INDEX);
}

} // namespace rmw::iox2

#endif
