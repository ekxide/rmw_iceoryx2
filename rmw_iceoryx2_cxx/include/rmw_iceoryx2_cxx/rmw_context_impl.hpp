// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_CONTEXT_IMPL_HPP_
#define RMW_IOX2_CONTEXT_IMPL_HPP_

#include "iox2/node.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"

#include <atomic>

namespace iox2_rmw
{

class RMW_PUBLIC ContextImpl
{
public:
    ContextImpl();

private:
    iox2::Node<iox2::ServiceType::Ipc> m_node;
};

} // namespace iox2_rmw

extern "C" {

struct rmw_context_impl_s
{
    std::atomic_bool is_initialized{false};
    iox2_rmw::ContextImpl* data;
};

} // extern "C"

#endif
