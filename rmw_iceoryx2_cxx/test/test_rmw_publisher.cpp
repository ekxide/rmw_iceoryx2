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

#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{
using namespace rmw::iox2::testing;

class RmwPublisherTest : public TestBase
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

    void initialize_test_node() {
        m_test_node = rmw_create_node(&context, "HypnoToad", "GloryTo");
    }

    void cleanup_test_node() {
        EXPECT_RMW_OK(rmw_destroy_node(m_test_node));
    }

    rmw_node_t* test_node() {
        return m_test_node;
    }

    template <typename MessageT>
    const rosidl_message_type_support_t* test_type_support() {
        return rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
    }

    const char* test_topic() {
        return m_test_topic;
    }

private:
    rmw_node_t* m_test_node;
    const char* m_test_topic = "Croak";
};

TEST_F(RmwPublisherTest, create_and_destroy) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto publisher = rmw_create_publisher(test_node(), test_type_support<Defaults>(), test_topic(), nullptr, nullptr);

    RMW_ASSERT_NE(publisher, nullptr);
    RMW_ASSERT_NE(publisher->implementation_identifier, nullptr);
    RMW_ASSERT_NE(publisher->data, nullptr);
    ASSERT_STREQ(publisher->topic_name, test_topic());
    RMW_ASSERT_TRUE(publisher->can_loan_messages);

    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

// TEST_F(RmwPublisherTest, borrow_and_return) {
//     using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
//
//     auto publisher = rmw_create_publisher(test_node(), test_type_support<Defaults>(), test_topic(), nullptr,
//     nullptr); auto loaned_message = nullptr;
//
//     EXPECT_RMW_OK(rmw_borrow_loaned_message(publisher, test_type_support(), loaned_message));
//     EXPECT_RMW_OK(rmw_return_loaned_message_from_publisher(publisher, loaned_message));
//
//     ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
// }

} // namespace
