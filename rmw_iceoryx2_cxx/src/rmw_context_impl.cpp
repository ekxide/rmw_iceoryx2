// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rmw_iceoryx2_cxx/rmw_context_impl.hpp"

rmw_context_impl_s::rmw_context_impl_s()
    : m_node{iox2::NodeBuilder()
                 .name(iox2::NodeName::create("ROS2 Context").expect("failed to create node name for context"))
                 .create<iox2::ServiceType::Ipc>()
                 .expect("failed to create iceroyx2 node for context")} {
}
