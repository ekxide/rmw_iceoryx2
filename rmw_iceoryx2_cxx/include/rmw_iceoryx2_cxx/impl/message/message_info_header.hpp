// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_MESSAGE_INFO_HEADER_HPP_
#define RMW_IOX2_MESSAGE_INFO_HEADER_HPP_

#include "iox2/type_name.hpp"
#include "rmw_iceoryx2_interoperability/rmw_iceoryx2_interoperability.h"

// Attaches the shared cross-language type name to the user-header struct so it matches across peers.
IOX2_DEFINE_TYPE_NAME(::rmw_iceoryx2_interoperability::MessageInfoHeader,
                      ::rmw_iceoryx2_interoperability::MESSAGE_INFO_HEADER_TYPE_NAME);

#endif // RMW_IOX2_MESSAGE_INFO_HEADER_HPP_
