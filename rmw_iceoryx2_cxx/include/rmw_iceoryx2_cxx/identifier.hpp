// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_IDENTIFIER_HPP_
#define RMW_IOX2_IDENTIFIER_HPP_

#include "rmw/visibility_control.h"

constexpr const char* rmw_iox2_identifier = "rmw_iceoryx2_cxx";

extern "C" {

RMW_PUBLIC
const char* rmw_get_implementation_identifier();

} // extern "C"

#endif // RMW_IOX2_IDENTIFIER_HPP_
