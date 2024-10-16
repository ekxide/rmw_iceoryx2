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
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwSubscriptionTest : public TestBase
{
protected:
    void SetUp() override {
        initialize_test_context();
        initialize_test_node();
    }

    void TearDown() override {
        cleanup_test_node();
        cleanup_test_context();
        print_rmw_errors();
    }
};

TEST_F(RmwSubscriptionTest, create_and_destroy) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto subscription =
        rmw_create_subscription(test_node(), test_type_support<Defaults>(), test_topic(), nullptr, nullptr);

    RMW_ASSERT_NE(subscription, nullptr);
    RMW_ASSERT_NE(subscription->implementation_identifier, nullptr);
    RMW_ASSERT_NE(subscription->data, nullptr);
    ASSERT_STREQ(subscription->topic_name, test_topic());
    RMW_ASSERT_TRUE(subscription->can_loan_messages);

    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
}

} // namespace
