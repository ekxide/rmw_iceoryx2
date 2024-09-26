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

#include "iox/optional.hpp"
#include "iox2/node.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"

extern "C" {

/// RMW expects exactly this type to be defined for context implementations.
class RMW_PUBLIC rmw_context_impl_s
{
public:
    explicit rmw_context_impl_s();

    static auto get(void* ptr) -> iox::optional<rmw_context_impl_s*>;

private:
    iox2::Node<iox2::ServiceType::Ipc> m_node;
};
}

namespace rmw::iox2
{

// Used throughout implementation to indicate it is C++
using ContextImpl = rmw_context_impl_s;

} // namespace rmw::iox2

#endif
