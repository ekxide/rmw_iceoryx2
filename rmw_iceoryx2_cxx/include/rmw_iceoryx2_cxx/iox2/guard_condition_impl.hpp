// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_GUARD_CONDITION_IMPL_HPP_
#define RMW_IOX2_GUARD_CONDITION_IMPL_HPP_

#include "iox/optional.hpp"
#include "iox2/notifier.hpp"
#include "iox2/port_factory_event.hpp"
#include "iox2/service_type.hpp"
#include "iox2/unique_port_id.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"

namespace rmw::iox2
{

class NodeImpl;
class GuardConditionImpl;

template <>
struct Error<GuardConditionImpl>
{
    using Type = GuardConditionError;
};

class RMW_PUBLIC GuardConditionImpl
{
    using RawIdType = ::iox2::RawIdType;
    using IdType = ::iox2::UniquePublisherId;
    using IceoryxNotifier = ::iox2::Notifier<::iox2::ServiceType::Ipc>;

public:
    using ErrorType = Error<GuardConditionImpl>::Type;

public:
    GuardConditionImpl(CreationLock,
                       iox::optional<ErrorType>& error,
                       NodeImpl& node,
                       const uint32_t context_id,
                       const uint32_t trigger_id);

    auto unique_id() -> const iox::optional<RawIdType>&;
    auto trigger_id() const -> uint32_t;
    auto service_name() const -> const std::string&;
    auto trigger() -> iox::expected<void, ErrorType>;

private:
    const uint32_t m_trigger_id;
    const std::string m_service_name;

    iox::optional<IdType> m_unique_id;
    iox::optional<IceoryxNotifier> m_notifier;
};

} // namespace rmw::iox2

#endif
