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
///          different parts of the system. This implementation uses a pipe that the
///          wait set waits on.
class RMW_PUBLIC UserGuardCondition : public GuardCondition
{
public:
    using ErrorType = Error<UserGuardCondition>::Type;

public:
    /// @brief Creates a new guard condition
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    UserGuardCondition(CreationLock, ::iox2::bb::Optional<ErrorType>& error);
    UserGuardCondition(const UserGuardCondition&) = delete;
    UserGuardCondition(UserGuardCondition&& other) noexcept;
    auto operator=(const UserGuardCondition&) -> UserGuardCondition& = delete;
    auto operator=(UserGuardCondition&& other) noexcept -> UserGuardCondition&;
    ~UserGuardCondition() override;

    auto trigger() -> ::iox2::bb::Expected<void, ErrorType> override;
    auto drain() -> bool override;
    auto file_descriptor() const -> ::iox2::FileDescriptorView override;

private:
    int m_read_end{-1};
    int m_write_end{-1};
    ::iox2::bb::Optional<::iox2::FileDescriptor> m_read_end_view;
};

/// @brief Guard condition of a node, triggered by every change to the graph in any process
class RMW_PUBLIC GraphGuardCondition : public GuardCondition
{
    using IceoryxNotifier = Iceoryx2::InterProcess::Notifier;
    using IceoryxListener = Iceoryx2::InterProcess::Listener;

public:
    using ErrorType = Error<GraphGuardCondition>::Type;

public:
    /// @brief Creates a new graph guard condition
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] iox2 The iceoryx2 handle that joins the graph event service
    GraphGuardCondition(CreationLock, ::iox2::bb::Optional<ErrorType>& error, Iceoryx2& iox2);

    auto trigger() -> ::iox2::bb::Expected<void, ErrorType> override;
    auto drain() -> bool override;
    auto file_descriptor() const -> ::iox2::FileDescriptorView override;

private:
    ::iox2::bb::Optional<IceoryxNotifier> m_iox2_notifier;
    ::iox2::bb::Optional<IceoryxListener> m_iox2_listener;
};

} // namespace rmw::iox2

#endif
