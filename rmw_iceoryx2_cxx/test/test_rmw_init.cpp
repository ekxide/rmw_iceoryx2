// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw_iceoryx2_cxx/rmw/identifier.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwInitTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(RmwInitTest, initialization_and_shutdown) {
    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    rmw_context_t context = rmw_get_zero_initialized_context();

    EXPECT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    EXPECT_RMW_OK(rmw_init(&init_options, &context));

    EXPECT_EQ(context.implementation_identifier, rmw_get_implementation_identifier());
    EXPECT_NE(context.impl, nullptr);

    EXPECT_RMW_OK(rmw_shutdown(&context));
    EXPECT_RMW_OK(rmw_context_fini(&context));
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

} // namespace
