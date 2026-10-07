// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_WAITSET_HPP_
#define RMW_IOX2_RUNTIME_WAITSET_HPP_

#include "iox2/bb/duration.hpp"
#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/legacy/variant.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/guard_condition.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/subscriber.hpp"

#include <functional>
#include <vector>

namespace rmw::iox2
{

/// An index used by the RMW to track entities attached to the waitset.
using RmwIndex = size_t;

/// Types of entities that the waitset is capable of waiting on.
enum class WaitableEntity { SUBSCRIBER, GUARD_CONDITION };

class WaitSet;

template <>
struct Error<WaitSet>
{
    using Type = WaitSetError;
};

/// @brief Implementation of the RMW wait set for iceoryx2
/// @details Waitable entities are tracked in upper layers (i.e. RCL) using indices.
///          This implementation maps these indices to iceoryx2 listeners for events signifying
///          work is available related to the associated the entities.
class RMW_PUBLIC WaitSet
{
    using Duration = ::iox2::bb::Duration;
    using ServiceName = ::iox2::ServiceName;
    using Guard = Iceoryx2::WaitSet::Guard;
    using AttachmentId = Iceoryx2::WaitSet::AttachmentId;
    using IceoryxWaitSet = Iceoryx2::WaitSet::Handle;
    using Waitable = ::iox2::legacy::variant<Subscriber*, GuardCondition*>;

    /// @brief Mapping from RMW index to a waitable entity.
    /// @details Allows for triggered listeners to be mapped back to the index that RMW uses for tracking
    struct RmwMapping
    {
        WaitableEntity waitable_type;
        RmwIndex rmw_index;
        Waitable entity;
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
        AttachmentDetails(Guard&& guard)
            : m_guard{std::move(guard)}
            , m_id{AttachmentId::from_guard(m_guard)} {
        }

        AttachmentDetails(Guard&& guard, const RmwMapping& staged_waitable)
            : m_guard{std::move(guard)}
            , m_id{AttachmentId::from_guard(m_guard)}
            , m_rmw_mapping{staged_waitable} {
        }

        auto id() const -> const AttachmentId& {
            return m_id;
        }

        auto mapping() const -> const RmwMapping& {
            return m_rmw_mapping;
        }

    private:
        Guard m_guard;
        AttachmentId m_id;
        RmwMapping m_rmw_mapping;
    };

    /// @brief Context for individual wait calls
    /// @details An instance of this is created for each wait call to track all attachments.
    struct WaitContext
    {
        ::iox2::bb::Optional<AttachmentDetails> attached_timeout;
        std::vector<AttachmentDetails> attached_listeners;
        std::vector<TriggeredWaitable> result;
    };

public:
    using ErrorType = Error<WaitSet>::Type;

public:
    /// @brief Constructor for the WaitSetImpl
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context to which this waitset is bound to. Must outlive the WaitSetImpl instance.
    WaitSet(CreationLock, ::iox2::bb::Optional<ErrorType>& error, Context& context);

    /// @brief Maps a guard condition to an RMW index
    /// @details The mapped guard condition is waited on in subsequent wait calls unless unmapped
    /// @param[in] rmw_index The index used to track the guard condition in the RMW
    /// @param[in] guard_condition The guard condition to be mapped
    auto map(RmwIndex rmw_index, GuardCondition& guard_condition) -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Maps a subscriber to an RMW index
    /// @details The mapped subscriber is waited on in subsequent wait calls unless unmapped
    /// @param[in] rmw_index The index used to track the subscriber in the RMW
    /// @param[in] subscribe The subscriber to be mapped
    auto map(RmwIndex rmw_index, Subscriber& subscriber) -> ::iox2::bb::Expected<void, ErrorType>;

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
    /// @param timeout Optional timeout after which waiting is stopped. If null waits indefinitely. If 0 does not wait
    ///                at all.
    /// @returns All triggered waitables.
    // TODO: Change return type. Copying the result vector is likely waste of runtime.
    auto wait(const ::iox2::bb::Optional<Duration>& timeout = ::iox2::bb::NULLOPT)
        -> ::iox2::bb::Expected<std::vector<TriggeredWaitable>, ErrorType>;

private:
    /// @brief Determine if provided timeout exists AND is zero
    /// @return True if timeout provided but it is zero i.e. process events without waiting
    auto zero_timeout(const ::iox2::bb::Optional<Duration>& timeout) const -> bool;

    /// @brief Determine if provided timeout is a nullopt
    /// @return True if timeout is a nullopt i.e. wait indefinitely
    auto no_timeout(const ::iox2::bb::Optional<Duration>& timeout) const -> bool;

    /// @brief Attach a timeout to the waitset.
    /// @details Creates an interval attachment to the waitset that will trigger after the specified duration
    /// @param[in] timeout The duration after which the timeout should trigger
    /// @param[out] ctx The context for the given wait call where the timeout attachment will be stored
    /// @return Success if the timeout was attached, error otherwise
    auto attach_timeout(const Duration& timeout, WaitContext& ctx) -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Attach all mapped listeners to the waitset.
    /// @details For each mapped listener, creates a notification attachment to the waitset and stores the details.
    ///          If any attachment fails, returns an error immediately. Waiting should not proceed in this case.
    /// @param[out] ctx The context for the given wait call where the notification attachment will be stored
    /// @return Success if all listeners were attached, error otherwise
    auto attach_mapped_listeners(WaitContext& ctx) -> ::iox2::bb::Expected<void, ErrorType>;

    /// @brief Process a triggered waitable entity
    /// @details Consumes the events from the listener of a triggered subscriber. A triggered guard condition keeps
    ///          its trigger until it is collected.
    /// @param[in] mapping The mapping of the triggered entity
    /// @return True if the entity is ready, error if the events could not be consumed
    auto process_trigger(const RmwMapping& mapping) -> ::iox2::bb::Expected<bool, ErrorType>;

    /// @brief Check whether a mapped entity is ready
    /// @details A subscriber is ready while it has samples, a guard condition once per trigger.
    /// @param[in] mapping The mapping of the entity
    /// @return True if the entity is ready
    auto is_ready(const RmwMapping& mapping) -> bool;

    /// @brief Collect all mapped entities that are ready
    /// @param[out] ctx The context for the given wait call where the ready entities are stored
    auto collect_ready(WaitContext& ctx) -> void;

    /// @brief Reference to RMW context that this WaitSet belongs to.
    auto context() -> Context&;

private:
    // `reference_wrapper` so the class remains move-constructible, which
    // iox2::bb::Optional's emplace path requires. Cannot be null by construction.
    std::reference_wrapper<Context> m_context;
    ::iox2::bb::Optional<IceoryxWaitSet> m_waitset;

    // Listeners staged to be waited on in the next wait call.
    // Maps the attachment to the index used in the RMW for tracking.
    std::vector<RmwMapping> m_mapping;
};

} // namespace rmw::iox2

#endif
