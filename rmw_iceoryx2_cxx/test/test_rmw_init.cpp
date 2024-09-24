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
#include "rmw_iceoryx2_cxx/rmw_context_impl.hpp"
#include "rmw_iceoryx2_cxx/rmw_identifier.hpp"
#include "test_helpers.hpp"

class RmwInitTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        allocator = rcutils_get_default_allocator();
        init_options = rmw_get_zero_initialized_init_options();
        context = rmw_get_zero_initialized_context();
    }

    void TearDown() override
    {
        // requires manual clean-up since it depends on the test
    }

    rcutils_allocator_t allocator;
    rmw_init_options_t init_options;
    rmw_context_t context;
};

TEST_F(RmwInitTest, InitializationAndShutdown)
{
    EXPECT_RMW_OK(rmw_init_options_init(&init_options, allocator));
    EXPECT_RMW_OK(rmw_init(&init_options, &context));

    EXPECT_EQ(context.implementation_identifier, rmw_get_implementation_identifier());
    EXPECT_EQ(context.instance_id, 0);
    EXPECT_NE(context.impl, nullptr);
    EXPECT_NE(context.impl->data, nullptr);

    EXPECT_RMW_OK(rmw_shutdown(&context));
    EXPECT_EQ(context.impl, nullptr);

    EXPECT_RMW_OK(rmw_context_fini(&context));
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitTest, InitializationWithNullArguments)
{
    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT, rmw_init_options_init(nullptr, allocator));
    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT, rmw_init(nullptr, &context));
    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT, rmw_init(&init_options, nullptr));
    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT, rmw_shutdown(nullptr));
}

TEST_F(RmwInitTest, MultipleInitialization)
{
    EXPECT_RMW_OK(rmw_init_options_init(&init_options, allocator));
    EXPECT_RMW_OK(rmw_init(&init_options, &context));

    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT, rmw_init(&init_options, &context));

    EXPECT_RMW_OK(rmw_shutdown(&context));
    EXPECT_RMW_OK(rmw_context_fini(&context));
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitTest, ShutdownWithoutInit)
{
    rmw_context_impl_s uninitialized{};
    context.implementation_identifier = rmw_get_implementation_identifier();
    context.impl = &uninitialized;
    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT, rmw_shutdown(&context));
}

TEST_F(RmwInitTest, ContextFinalization)
{
    EXPECT_RMW_OK(rmw_init_options_init(&init_options, allocator));
    EXPECT_RMW_OK(rmw_init(&init_options, &context));

    EXPECT_RMW_OK(rmw_shutdown(&context));
    EXPECT_RMW_OK(rmw_context_fini(&context));

    EXPECT_EQ(context.implementation_identifier, nullptr);
    EXPECT_EQ(context.instance_id, 0);
    EXPECT_EQ(context.impl, nullptr);
}
