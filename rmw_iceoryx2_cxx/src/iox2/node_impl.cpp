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
NodeImpl::NodeImpl(const std::string& name)
    : m_node{::iox2::NodeBuilder()
                 .name(::iox2::NodeName::create(name.c_str()).expect("failed to create node name"))
                 .create<::iox2::ServiceType::Ipc>()
                 .expect("failed to create iceoryx2 node")} {
}

auto NodeImpl::node_name() const -> const std::string& {
    return m_node_name;
}

auto NodeImpl::as_iox2() -> IceoryxNode& {
    return m_node;
}

} // namespace rmw::iox2
