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
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{
using namespace rmw::iox2::testing;

class RmwPublishSubscribeTest : public TestBase
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

// ----- Copy API ----- //

TEST_F(RmwPublishSubscribeTest, take_self_contained_no_new_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    void* subscriber_loan = nullptr;
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &subscriber_loan, &taken, nullptr));
    ASSERT_FALSE(taken);
}

TEST_F(RmwPublishSubscribeTest, take_self_contained_one_new_message) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    auto send_payload = Defaults{};
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    void* recv_payload = malloc(sizeof(Defaults));

    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, recv_payload, &taken, nullptr));
    ASSERT_NE(recv_payload, nullptr);
    ASSERT_TRUE(taken);

    ASSERT_EQ(*reinterpret_cast<Defaults*>(recv_payload), send_payload);

    free(recv_payload);
}

TEST_F(RmwPublishSubscribeTest, take_non_self_contained_no_new_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    void* subscriber_loan = nullptr;
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &subscriber_loan, &taken, nullptr));
    ASSERT_FALSE(taken);
}

TEST_F(RmwPublishSubscribeTest, take_non_self_contained_one_new_message) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    auto send_payload = Strings{};
    send_payload.string_value = "GloryToHypnoToad";
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    void* recv_payload = malloc(sizeof(Strings));
    new (recv_payload) Strings{};

    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, recv_payload, &taken, nullptr));
    ASSERT_NE(recv_payload, nullptr);
    ASSERT_TRUE(taken);

    ASSERT_EQ(*reinterpret_cast<Strings*>(recv_payload), send_payload);

    free(recv_payload);
}

// ----- Loan API ----- //

TEST_F(RmwPublishSubscribeTest, take_loan_self_contained_no_new_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    void* subscriber_loan = nullptr;
    bool taken{false};
    ASSERT_RMW_OK(rmw_take_loaned_message(subscription, &subscriber_loan, &taken, nullptr));
    ASSERT_FALSE(taken);
}

TEST_F(RmwPublishSubscribeTest, borrow_loan_non_self_contained_no_new_messages_unsupported) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);

    void* publisher_loan = nullptr;
    ASSERT_RMW_ERR(RMW_RET_INVALID_ARGUMENT,
                   rmw_borrow_loaned_message(publisher, test_type_support<Strings>(), &publisher_loan));
}

TEST_F(RmwPublishSubscribeTest, take_loan_non_self_contained_no_new_messages_unsupported) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    void* subscriber_loan = nullptr;
    bool taken{false};
    ASSERT_RMW_ERR(RMW_RET_UNSUPPORTED, rmw_take_loaned_message(subscription, &subscriber_loan, &taken, nullptr));
}

TEST_F(RmwPublishSubscribeTest, take_loan_self_contained_one_new_message) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    void* publisher_loan = nullptr;
    ASSERT_RMW_OK(rmw_borrow_loaned_message(publisher, test_type_support<Defaults>(), &publisher_loan));
    new (publisher_loan) Defaults{};
    ASSERT_RMW_OK(rmw_publish_loaned_message(publisher, publisher_loan, nullptr));

    void* subscriber_loan = nullptr;
    bool taken{false};
    ASSERT_RMW_OK(rmw_take_loaned_message(subscription, &subscriber_loan, &taken, nullptr));
    ASSERT_NE(subscriber_loan, nullptr);
    ASSERT_TRUE(taken);

    ASSERT_EQ(*reinterpret_cast<Defaults*>(subscriber_loan), Defaults{});

    ASSERT_RMW_OK(rmw_return_loaned_message_from_subscription(subscription, subscriber_loan));
}

// ----- Serialized Message API ----- //

TEST_F(RmwPublishSubscribeTest, take_serialized_no_new_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    rmw_serialized_message_t serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Strings), &test_allocator()));

    bool taken{false};
    ASSERT_RMW_OK(rmw_take_serialized_message(subscription, &serialized_msg, &taken, nullptr));
    ASSERT_FALSE(taken);
}

TEST_F(RmwPublishSubscribeTest, take_serialized_one_new_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    // Create publisher and subscriber
    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    // Serialized input message
    Strings input{};
    input.string_value = "GloryToHypnoToad";

    rmw_serialized_message_t input_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&input_serialized_msg, sizeof(Strings), &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &input_serialized_msg));

    // Publish serialized message
    ASSERT_RMW_OK(rmw_publish_serialized_message(publisher, &input_serialized_msg, nullptr));

    // Take serialized message
    rmw_serialized_message_t output_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&output_serialized_msg, sizeof(Strings), &test_allocator()));
    bool taken{false};
    ASSERT_RMW_OK(rmw_take_serialized_message(subscription, &output_serialized_msg, &taken, nullptr));
    ASSERT_TRUE(taken);

    // Deserialize output message
    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&output_serialized_msg, test_type_support<Strings>(), &output));

    // Verify
    ASSERT_EQ(input, output);
}

TEST_F(RmwPublishSubscribeTest, take_serialized_many_new_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    constexpr uint64_t NUM_MESSAGES = 3;

    // Create publisher and subscriber
    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    // Serialized input message
    Strings input{};
    input.string_value = "GloryToHypnoToad";

    rmw_serialized_message_t input_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&input_serialized_msg, sizeof(Strings), &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &input_serialized_msg));

    // Publish serialized message many times
    for (size_t i = 0; i < NUM_MESSAGES; ++i) {
        ASSERT_RMW_OK(rmw_publish_serialized_message(publisher, &input_serialized_msg, nullptr));
    }

    // Take serialized messages many times
    for (size_t i = 0; i < NUM_MESSAGES; ++i) {
        rmw_serialized_message_t output_serialized_msg{};
        ASSERT_RMW_OK(rmw_serialized_message_init(&output_serialized_msg, sizeof(Strings), &test_allocator()));
        bool taken{false};
        ASSERT_RMW_OK(rmw_take_serialized_message(subscription, &output_serialized_msg, &taken, nullptr));
        ASSERT_TRUE(taken);

        // Deserialize output message
        Strings output{};
        ASSERT_RMW_OK(rmw_deserialize(&output_serialized_msg, test_type_support<Strings>(), &output));

        // Verify
        ASSERT_EQ(input, output);
    }
}

} // namespace
