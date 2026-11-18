// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_QOS_PORT_NAME_HPP_
#define RMW_IOX2_QOS_PORT_NAME_HPP_

#include "iox2/bb/expected.hpp"
#include "iox2/bb/optional.hpp"
#include "iox2/bb/semantic_string.hpp"
#include "iox2/port_name.hpp"
#include "rmw/visibility_control.h"
#include "rmw_iceoryx2_cxx/impl/qos/qos.hpp"

namespace rmw::iox2::port_name
{

/// @brief Encodes the QoS of an endpoint as the name of its iceoryx2 port
/// @param[in] qos The QoS of the endpoint
/// @return The port name, or the error of creating it
RMW_PUBLIC auto encode(const Qos& qos) -> ::iox2::bb::Expected<::iox2::PortName, ::iox2::bb::SemanticStringError>;

/// @brief Decodes the QoS of an endpoint from the name of its iceoryx2 port
/// @param[in] name The name of the port
/// @return The QoS, or nothing when the name holds no encoded QoS
RMW_PUBLIC auto decode(::iox2::PortNameView name) -> ::iox2::bb::Optional<Qos>;

} // namespace rmw::iox2::port_name

#endif // RMW_IOX2_QOS_PORT_NAME_HPP_
