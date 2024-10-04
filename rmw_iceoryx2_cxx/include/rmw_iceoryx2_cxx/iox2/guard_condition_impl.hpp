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
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

namespace rmw::iox2
{

class RMW_PUBLIC GuardConditionImpl
{
    using IceoryxNotifier = ::iox2::Notifier<::iox2::ServiceType::Ipc>;

public:
    GuardConditionImpl(NodeImpl& node, const uint32_t context_id, const uint32_t guard_condition_id);

    uint32_t id() const;
    bool trigger(const iox::optional<size_t>& id = iox::nullopt);

private:
    const uint32_t m_id;
    iox::optional<IceoryxNotifier> m_notifier;
};

} // namespace rmw::iox2

#endif
