// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RUNTIME_GUARD_CONDITION_HPP_
#define RMW_IOX2_RUNTIME_GUARD_CONDITION_HPP_

#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/file_descriptor.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"

class rmw_context_impl_s;

namespace rmw::iox2
{

using Context = rmw_context_impl_s;

class Node;
class GuardCondition;
class UserGuardCondition;
class GraphGuardCondition;

template <>
struct Error<GuardCondition>
{
    using Type = GuardConditionError;
};

template <>
struct Error<UserGuardCondition>
{
    using Type = GuardConditionError;
};

template <>
struct Error<GraphGuardCondition>
{
    using Type = GuardConditionError;
};

/// @brief Interface of the guard conditions that the RMW triggers and waits on
class RMW_PUBLIC GuardCondition
{
public:
    using ErrorType = Error<GuardCondition>::Type;

    GuardCondition() = default;
    GuardCondition(const GuardCondition&) = default;
    GuardCondition(GuardCondition&&) = default;
    auto operator=(const GuardCondition&) -> GuardCondition& = default;
    auto operator=(GuardCondition&&) -> GuardCondition& = default;
    virtual ~GuardCondition() = default;

    /// @brief Triggers the guard condition
    /// @return Error if the trigger failed
    virtual auto trigger() -> ::iox2::bb::Expected<void, ErrorType> = 0;

    /// @brief Consume the triggers received since the last call
    /// @return True if the guard condition was triggered since the last call
    virtual auto drain() -> bool = 0;

    /// @brief Get the file descriptor to wait on for triggers
    /// @return The file descriptor of the listener that receives the triggers
    virtual auto file_descriptor() const -> ::iox2::FileDescriptorView = 0;
};

/// @brief Implementation of the RMW guard condition for iceoryx2
/// @details A guard condition is a synchronization primitive that can be used to
///          wake up a waiting thread. It is used in ROS 2 to signal events between
///          different parts of the system. This implementation uses an iceoryx2
///          notifier to implement the guard condition functionality.
class RMW_PUBLIC UserGuardCondition : public GuardCondition
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniquePublisherId;
    using IceoryxNotifier = Iceoryx2::Local::Notifier;
    using IceoryxListener = Iceoryx2::Local::Listener;

public:
    using ErrorType = Error<UserGuardCondition>::Type;

public:
    /// @brief Creates a new guard condition
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context to associate the guard condition with
    UserGuardCondition(CreationLock, ::iox2::bb::Optional<ErrorType>& error, Context& context);

    /// @brief Get the unique id of the guard condition
    /// @return The unique id or empty optional if failing to retrieve it from iceoryx2
    auto unique_id() -> const ::iox2::bb::Optional<RawIdType>&;

    /// @brief Get the trigger id of the guard condition
    /// @return The trigger id
    auto trigger_id() const -> uint32_t;

    /// @brief Get the iceoryx2 service name of the guard condition
    /// @return The service name
    auto service_name() const -> const std::string&;

    auto trigger() -> ::iox2::bb::Expected<void, ErrorType> override;
    auto drain() -> bool override;
    auto file_descriptor() const -> ::iox2::FileDescriptorView override;

private:
    uint32_t m_trigger_id;
    std::string m_service_name;

    ::iox2::bb::Optional<IdType> m_iox2_unique_id;
    ::iox2::bb::Optional<IceoryxNotifier> m_iox2_notifier;
    ::iox2::bb::Optional<IceoryxListener> m_iox2_listener;
};

/// @brief Guard condition of a node, triggered by every change to the graph in any process
class RMW_PUBLIC GraphGuardCondition : public GuardCondition
{
    using IceoryxService = ::iox2::PortFactoryEvent<Iceoryx2::ServiceType::Ipc>;
    using IceoryxNotifier = Iceoryx2::InterProcess::Notifier;
    using IceoryxListener = Iceoryx2::InterProcess::Listener;

public:
    using ErrorType = Error<GraphGuardCondition>::Type;

public:
    /// @brief Creates a new graph guard condition
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] graph_service The graph event service the guard condition notifies and listens to
    GraphGuardCondition(CreationLock, ::iox2::bb::Optional<ErrorType>& error, IceoryxService& graph_service);

    auto trigger() -> ::iox2::bb::Expected<void, ErrorType> override;
    auto drain() -> bool override;
    auto file_descriptor() const -> ::iox2::FileDescriptorView override;

private:
    ::iox2::bb::Optional<IceoryxNotifier> m_iox2_notifier;
    ::iox2::bb::Optional<IceoryxListener> m_iox2_listener;
};

} // namespace rmw::iox2

#endif
