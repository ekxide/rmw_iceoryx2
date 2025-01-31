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

#include "iox/optional.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/common/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/impl/common/error.hpp"
#include "rmw_iceoryx2_cxx/impl/middleware/iceoryx2.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"

namespace rmw::iox2
{

class Node;
class GuardCondition;

template <>
struct Error<GuardCondition>
{
    using Type = GuardConditionError;
};

/// @brief Implementation of the RMW guard condition for iceoryx2
/// @details A guard condition is a synchronization primitive that can be used to
///          wake up a waiting thread. It is used in ROS 2 to signal events between
///          different parts of the system. This implementation uses an iceoryx2
///          notifier to implement the guard condition functionality.
class RMW_PUBLIC GuardCondition
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniquePublisherId;
    using IceoryxNotifier = Iceoryx2::Local::Notifier;

public:
    using ErrorType = Error<GuardCondition>::Type;

public:
    /// @brief Creates a new guard condition
    /// @param[in] lock Creation lock to restrict construction to creation functions
    /// @param[out] error Optional error that is set if construction fails
    /// @param[in] context The context to associate the guard condition with
    GuardCondition(CreationLock, iox::optional<ErrorType>& error, Context& context);

    /// @brief Get the unique id of the guard condition
    /// @return The unique id or empty optional if failing to retrieve it from iceoryx2
    auto unique_id() -> const iox::optional<RawIdType>&;

    /// @brief Get the trigger id of the guard condition
    /// @return The trigger id
    auto trigger_id() const -> uint32_t;

    /// @brief Get the iceoryx2 service name of the guard condition
    /// @return The service name
    auto service_name() const -> const std::string&;

    /// @brief Triggers the guard condition
    /// @return Error if trigger via iceoryx2 failed
    auto trigger() -> iox::expected<void, ErrorType>;

private:
    const uint32_t m_trigger_id;
    const std::string m_service_name;

    iox::optional<IdType> m_iox2_unique_id;
    iox::optional<IceoryxNotifier> m_iox2_notifier;
};

} // namespace rmw::iox2

#endif
