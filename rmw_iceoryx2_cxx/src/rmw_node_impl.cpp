// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/rmw_node_impl.hpp"

namespace iox2_rmw
{

NodeImpl::NodeImpl(const char* name)
    : m_node{iox2::NodeBuilder()
                 .name(iox2::NodeName::create(name).expect("failed to create node name"))
                 .create<iox2::ServiceType::Ipc>()
                 .expect("failed to create iceoryx2 node")} {
}

auto get(void* ptr) -> iox::optional<NodeImpl*> {
    if (!ptr) {
        return iox::nullopt;
    }
    return reinterpret_cast<NodeImpl*>(ptr);
};

auto NodeImpl::impl() -> const iox2::Node<iox2::ServiceType::Ipc>& {
    return m_node;
};

} // namespace iox2_rmw
