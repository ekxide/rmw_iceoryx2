// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

namespace rmw::iox2
{

// TODO: fallable constructors
//       make underlying type an optional, set RMW error on failure
NodeImpl::NodeImpl(const std::string name)
    : m_node{::iox2::NodeBuilder()
                 .name(::iox2::NodeName::create(name.c_str()).expect("failed to create node name"))
                 .create<::iox2::ServiceType::Ipc>()
                 .expect("failed to create iceoryx2 node")} {
}

auto NodeImpl::create_notifier(const std::string name) -> Notifier {
    auto service_name = ::iox2::ServiceName::create(name.c_str()).expect("TODO: propagate");
    auto service = m_node.service_builder(service_name).event().open_or_create().expect("TODO: propagate");
    auto notifier = service.notifier_builder().create().expect("TODO: propagate");

    return notifier;
}

auto NodeImpl::create_listener(const std::string name) -> Listener {
    auto service_name = ::iox2::ServiceName::create(name.c_str()).expect("TODO: propagate");
    auto service = m_node.service_builder(service_name).event().open_or_create().expect("TODO: propagate");
    auto listener = service.listener_builder().create().expect("TODO: propagate");

    return listener;
}

} // namespace rmw::iox2
