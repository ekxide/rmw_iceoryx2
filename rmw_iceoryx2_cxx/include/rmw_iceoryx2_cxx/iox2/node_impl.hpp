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

#include "iox/optional.hpp"
#include "iox2/node.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/creation_lock.hpp"
#include "rmw_iceoryx2_cxx/error.hpp"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"

namespace rmw::iox2
{

class NodeImpl;

template <>
struct Error<NodeImpl>
{
    using Type = NodeError;
};

class RMW_PUBLIC NodeImpl
{
    using IceoryxNode = ::iox2::Node<::iox2::ServiceType::Ipc>;

public:
    using ErrorType = Error<NodeImpl>::Type;

public:
    NodeImpl(CreationLock, iox::optional<ErrorType>& error, const std::string& name);

    auto node_name() const -> const std::string&;

    auto as_iox2() -> IceoryxNode&;

private:
    const std::string m_node_name;
    iox::optional<IceoryxNode> m_node;
};

} // namespace rmw::iox2

#endif
