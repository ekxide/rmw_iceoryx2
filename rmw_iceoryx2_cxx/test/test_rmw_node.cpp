// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/rmw_identifier.hpp"
#include "test_helpers.hpp"

class RmwNodeTest : public ::testing::Test
{
protected:
    void SetUp() override {
        allocator = rcutils_get_default_allocator();
        init_options = rmw_get_zero_initialized_init_options();
        ASSERT_RMW_OK(rmw_init_options_init(&init_options, allocator));
        context = rmw_get_zero_initialized_context();
        ASSERT_RMW_OK(rmw_init(&init_options, &context));
    }

    void TearDown() override {
        ASSERT_RMW_OK(rmw_shutdown(&context));
        ASSERT_RMW_OK(rmw_context_fini(&context));
        ASSERT_RMW_OK(rmw_init_options_fini(&init_options));
    }

protected:
    const char* test_name = "Perception";
    const char* test_namespace = "ADAS";
    rcutils_allocator_t allocator;
    rmw_init_options_t init_options;
    rmw_context_t context;
};

TEST_F(RmwNodeTest, CreateAndDestroy) {
    auto node = rmw_create_node(&context, test_name, test_namespace);

    ASSERT_EQ(node->implementation_identifier, rmw_get_implementation_identifier());
    ASSERT_STREQ(node->name, test_name);
    ASSERT_STREQ(node->namespace_, test_namespace);
    ASSERT_NE(node->data, nullptr);

    ASSERT_RMW_OK(rmw_destroy_node(node));
    ASSERT_EQ(node->name, nullptr);
    ASSERT_EQ(node->namespace_, nullptr);
    ASSERT_EQ(node->data, nullptr);
}

TEST_F(RmwNodeTest, CreateWithInvalidArguments) {
    EXPECT_NULLPTR_WITH_RMW_ERR(rmw_create_node(nullptr, test_name, test_namespace));
    EXPECT_NULLPTR_WITH_RMW_ERR(rmw_create_node(&context, nullptr, test_namespace));
    EXPECT_NULLPTR_WITH_RMW_ERR(rmw_create_node(&context, test_name, nullptr));
}
