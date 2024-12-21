// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_SERIALIZATION_FORMAT_HPP_
#define RMW_IOX2_SERIALIZATION_FORMAT_HPP_

#include "rmw/visibility_control.h"

extern "C" {

RMW_PUBLIC
extern const char* const rmw_iox2_serialization_format;

const char* rmw_get_serialization_format(void);

} // extern "C"

#endif // RMW_IOX2_SERIALIZATION_FORMAT_HPP_
