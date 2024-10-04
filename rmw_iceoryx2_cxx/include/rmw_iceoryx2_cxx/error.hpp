// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#ifndef RMW_IOX2_ERROR_HPP_
#define RMW_IOX2_ERROR_HPP_

#include <cstdint>

namespace rmw::iox2
{

enum class IceoryxError : uint8_t { ERROR };
enum class MemoryError : uint8_t { ALLOCATION, CONSTRUCTION, CAST };

} // namespace rmw::iox2

#endif
