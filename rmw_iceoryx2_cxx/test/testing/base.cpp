// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "base.hpp"

namespace rmw::iox2::testing::names
{

std::string test_handle(uint32_t test_id) {
    return std::string("::test_node::" + std::to_string(test_id));
}

} // namespace rmw::iox2::testing::names
