// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_RMW_NODE_HPP_
#define RMW_IOX2_RMW_NODE_HPP_

#include "iox2/bb/optional.hpp"
#include "rmw/types.h"
#include "rmw_iceoryx2_cxx/impl/runtime/node.hpp"

namespace rmw::iox2
{

/// @brief The data of an rmw node: the node and the rmw handle of its graph guard condition
struct NodeData
{
    ::iox2::bb::Optional<Node> node;
    rmw_guard_condition_t graph_guard_condition{};
};

} // namespace rmw::iox2

#endif // RMW_IOX2_RMW_NODE_HPP_
