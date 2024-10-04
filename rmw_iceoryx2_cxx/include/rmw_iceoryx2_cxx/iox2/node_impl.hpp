// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_NODE_IMPL_HPP_
#define RMW_IOX2_NODE_IMPL_HPP_

#include "iox2/listener.hpp"
#include "iox2/node.hpp"
#include "iox2/notifier.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"

namespace rmw::iox2
{

class RMW_PUBLIC NodeImpl
{
    using Node = ::iox2::Node<::iox2::ServiceType::Ipc>;
    using Notifier = ::iox2::Notifier<::iox2::ServiceType::Ipc>;
    using Listener = ::iox2::Listener<::iox2::ServiceType::Ipc>;

public:
    explicit NodeImpl(const std::string& name);

    auto create_notifier(const std::string name) -> Notifier;
    auto create_listener(const std::string name) -> Listener;

private:
    Node m_node;
};

} // namespace rmw::iox2

#endif
