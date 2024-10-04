// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_PUBLISHER_IMPL_HPP_
#define RMW_IOX2_PUBLISHER_IMPL_HPP_

#include "iox/optional.hpp"
#include "iox/slice.hpp"
#include "iox2/publisher.hpp"
#include "iox2/service_type.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/iox2/node_impl.hpp"

namespace rmw::iox2
{

class RMW_PUBLIC PublisherImpl
{
    using IceoryxPublisher = ::iox2::Publisher<::iox2::ServiceType::Ipc, ::iox::Slice<uint8_t>, void>;

public:
    explicit PublisherImpl(NodeImpl& node, const std::string& name);

private:
    iox::optional<IceoryxPublisher> m_publisher;
};

} // namespace rmw::iox2

#endif
