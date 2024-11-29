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
#include "rmw_iceoryx2_cxx/identifier.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwNodeTest : public TestBase
{
protected:
    void SetUp() override {
        initialize_test_context();
    }

    void TearDown() override {
        cleanup_test_context();
        print_rmw_errors();
    }

protected:
    const char* test_name = "Perception";
    const char* test_namespace = "/ADAS";
};

TEST_F(RmwNodeTest, create_and_destroy) {
    auto node = rmw_create_node(test_context(), test_name, test_namespace);
    ASSERT_NE(node, nullptr);

    ASSERT_EQ(node->implementation_identifier, rmw_get_implementation_identifier());
    ASSERT_STREQ(node->name, test_name);
    ASSERT_STREQ(node->namespace_, test_namespace);
    ASSERT_NE(node->data, nullptr);

    ASSERT_RMW_OK(rmw_destroy_node(node));
    ASSERT_EQ(node->name, nullptr);
    ASSERT_EQ(node->namespace_, nullptr);
    ASSERT_EQ(node->data, nullptr);
}

} // namespace
