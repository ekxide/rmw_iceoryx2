// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "iox2/log.hpp"
#include <gtest/gtest.h>

namespace rmw::iox2::testing
{

class TestBase : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {
        ::iox2::set_log_level(::iox2::LogLevel::ERROR);
    }
};

} // namespace rmw::iox2::testing
