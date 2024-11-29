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

class RmwPublisherTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }
};

TEST_F(RmwPublisherTest, create_and_destroy) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());

    RMW_ASSERT_NE(publisher, nullptr);
    RMW_ASSERT_NE(publisher->implementation_identifier, nullptr);
    RMW_ASSERT_NE(publisher->data, nullptr);
    ASSERT_STREQ(publisher->topic_name, create_test_topic().c_str());
    RMW_ASSERT_TRUE(publisher->can_loan_messages);
}

TEST_F(RmwPublisherTest, borrow_and_return) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    void* loaned_message = nullptr;

    EXPECT_RMW_OK(rmw_borrow_loaned_message(publisher, test_type_support<Defaults>(), &loaned_message));
    EXPECT_RMW_OK(rmw_return_loaned_message_from_publisher(publisher, loaned_message));
}

} // namespace
