// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/node.hpp"
#include "iox2/service_name.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/time.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/bounded_sequences.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <array>
#include <cstdlib>
#include <string>

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

// ---------------------------------------------------------------------------
// Copy API
// ---------------------------------------------------------------------------

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

TEST_F(RmwPublishSubscribeTest, take_non_self_contained_one_new_message_c_type_support) {
    const auto* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(rmw_iceoryx2_cxx_test_msgs, msg, Strings);

    auto* publisher = create_publisher(type_support, create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_subscriber(type_support, create_test_topic());
    ASSERT_NE(subscription, nullptr);

    rmw_iceoryx2_cxx_test_msgs__msg__Strings send_payload;
    ASSERT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__Strings__init(&send_payload));
    ASSERT_TRUE(rosidl_runtime_c__String__assign(&send_payload.string_value, "GloryToHypnoToad"));
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    rmw_iceoryx2_cxx_test_msgs__msg__Strings recv_payload;
    ASSERT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__Strings__init(&recv_payload));
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &recv_payload, &taken, nullptr));
    ASSERT_TRUE(taken);
    EXPECT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__Strings__are_equal(&send_payload, &recv_payload));

    rmw_iceoryx2_cxx_test_msgs__msg__Strings__fini(&recv_payload);
    rmw_iceoryx2_cxx_test_msgs__msg__Strings__fini(&send_payload);
}

TEST_F(RmwPublishSubscribeTest, failed_publish_returns_the_loan) {
    const auto* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(rmw_iceoryx2_cxx_test_msgs, msg, BoundedSequences);

    auto* publisher = create_publisher(type_support, create_test_topic());
    ASSERT_NE(publisher, nullptr);

    rmw_iceoryx2_cxx_test_msgs__msg__BoundedSequences message;
    ASSERT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__BoundedSequences__init(&message));

    // bool_values is bounded to 3 elements, so serialization fails after the sample is loaned.
    // Fail more often than the publisher can hold loans at once.
    ASSERT_TRUE(rosidl_runtime_c__boolean__Sequence__init(&message.bool_values, 4));
    for (int i = 0; i < 3; ++i) {
        EXPECT_RMW_ERR(RMW_RET_ERROR, rmw_publish(publisher, &message, nullptr));
    }

    message.bool_values.size = 3;
    EXPECT_RMW_OK(rmw_publish(publisher, &message, nullptr));

    rmw_iceoryx2_cxx_test_msgs__msg__BoundedSequences__fini(&message);
}

TEST_F(RmwPublishSubscribeTest, failed_take_returns_the_loan) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    std::array<uint8_t, 4> garbage{0xFF, 0xFF, 0xFF, 0xFF};
    rmw_serialized_message_t garbage_message{garbage.data(), garbage.size(), garbage.size(), test_allocator()};
    Strings output{};
    bool taken{false};
    for (int i = 0; i < 3; ++i) {
        ASSERT_RMW_OK(rmw_publish_serialized_message(publisher, &garbage_message, nullptr));
        EXPECT_RMW_ERR(RMW_RET_ERROR, rmw_take(subscription, &output, &taken, nullptr));
    }

    Strings input{};
    input.string_value = "GloryToHypnoToad";
    ASSERT_RMW_OK(rmw_publish(publisher, &input, nullptr));
    ASSERT_RMW_OK(rmw_take(subscription, &output, &taken, nullptr));
    ASSERT_TRUE(taken);
    ASSERT_EQ(input, output);
}

TEST_F(RmwPublishSubscribeTest, failed_take_serialized_message_returns_the_loan) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    Strings input{};
    input.string_value = "GloryToHypnoToad";

    auto failing_allocator = rcutils_get_default_allocator();
    failing_allocator.reallocate = [](void*, size_t, void*) -> void* { return nullptr; };
    rmw_serialized_message_t too_small{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&too_small, 1, &failing_allocator));
    bool taken{false};
    for (int i = 0; i < 3; ++i) {
        ASSERT_RMW_OK(rmw_publish(publisher, &input, nullptr));
        EXPECT_RMW_ERR(RMW_RET_ERROR, rmw_take_serialized_message(subscription, &too_small, &taken, nullptr));
    }
    ASSERT_RMW_OK(rmw_serialized_message_fini(&too_small));

    ASSERT_RMW_OK(rmw_publish(publisher, &input, nullptr));
    rmw_serialized_message_t output_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&output_serialized_msg, sizeof(Strings), &test_allocator()));
    ASSERT_RMW_OK(rmw_take_serialized_message(subscription, &output_serialized_msg, &taken, nullptr));
    ASSERT_TRUE(taken);
    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&output_serialized_msg, test_type_support<Strings>(), &output));
    ASSERT_EQ(input, output);
    ASSERT_RMW_OK(rmw_serialized_message_fini(&output_serialized_msg));
}

TEST_F(RmwPublishSubscribeTest, take_non_self_contained_message_larger_than_struct) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    // A string long enough that the serialized payload exceeds the message struct size, forcing the
    // byte-granular loan to grow past its initial slice length.
    auto send_payload = Strings{};
    send_payload.string_value = std::string(sizeof(Strings) * 4, 'x');
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    void* recv_payload = malloc(sizeof(Strings));
    new (recv_payload) Strings{};

    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, recv_payload, &taken, nullptr));
    ASSERT_TRUE(taken);

    ASSERT_EQ(*reinterpret_cast<Strings*>(recv_payload), send_payload);

    free(recv_payload);
}

// ---------------------------------------------------------------------------
// Loan API
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Serialized message API
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Message info
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, take_with_info_populates_message_info) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    rcutils_time_point_value_t before_publish{0};
    ASSERT_EQ(rcutils_system_time_now(&before_publish), RCUTILS_RET_OK);

    auto send_payload = Defaults{};
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    void* recv_payload = malloc(sizeof(Defaults));
    bool taken{false};
    rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
    ASSERT_RMW_OK(rmw_take_with_info(subscription, recv_payload, &taken, &message_info, nullptr));
    ASSERT_TRUE(taken);

    rcutils_time_point_value_t after_take{0};
    ASSERT_EQ(rcutils_system_time_now(&after_take), RCUTILS_RET_OK);

    // source_timestamp is stamped at publish, received_timestamp at take; both fall within the
    // bounding window and are ordered.
    EXPECT_GE(message_info.source_timestamp, before_publish);
    EXPECT_LE(message_info.source_timestamp, message_info.received_timestamp);
    EXPECT_LE(message_info.received_timestamp, after_take);
    EXPECT_EQ(message_info.publication_sequence_number, 0u);
    EXPECT_FALSE(message_info.from_intra_process);

    free(recv_payload);
}

TEST_F(RmwPublishSubscribeTest, take_with_info_populates_publisher_gid) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    auto send_payload = Defaults{};
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    auto recv_payload = Defaults{};
    bool taken{false};
    rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
    ASSERT_RMW_OK(rmw_take_with_info(subscription, &recv_payload, &taken, &message_info, nullptr));
    ASSERT_TRUE(taken);

    rmw_gid_t publisher_gid{};
    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher, &publisher_gid));
    bool equal{false};
    ASSERT_RMW_OK(rmw_compare_gids_equal(&publisher_gid, &message_info.publisher_gid, &equal));
    EXPECT_TRUE(equal);
}

TEST_F(RmwPublishSubscribeTest, take_with_info_populates_publisher_gid_non_self_contained) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    auto send_payload = Strings{};
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    auto recv_payload = Strings{};
    bool taken{false};
    rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
    ASSERT_RMW_OK(rmw_take_with_info(subscription, &recv_payload, &taken, &message_info, nullptr));
    ASSERT_TRUE(taken);

    rmw_gid_t publisher_gid{};
    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher, &publisher_gid));
    bool equal{false};
    ASSERT_RMW_OK(rmw_compare_gids_equal(&publisher_gid, &message_info.publisher_gid, &equal));
    EXPECT_TRUE(equal);
}

TEST_F(RmwPublishSubscribeTest, take_sequence_takes_all_available_messages) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    constexpr size_t COUNT = 3;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    for (size_t i = 0; i < COUNT; ++i) {
        auto send_payload = Defaults{};
        send_payload.int64_value = static_cast<int64_t>(i);
        ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));
    }

    std::array<Defaults, COUNT> messages;
    auto allocator = rcutils_get_default_allocator();
    auto sequence = rmw_get_zero_initialized_message_sequence();
    ASSERT_RMW_OK(rmw_message_sequence_init(&sequence, COUNT, &allocator));
    for (size_t i = 0; i < COUNT; ++i) {
        sequence.data[i] = &messages[i];
    }
    auto info_sequence = rmw_get_zero_initialized_message_info_sequence();
    ASSERT_RMW_OK(rmw_message_info_sequence_init(&info_sequence, COUNT, &allocator));

    size_t taken{0};
    ASSERT_RMW_OK(rmw_take_sequence(subscription, COUNT, &sequence, &info_sequence, &taken, nullptr));
    EXPECT_EQ(taken, COUNT);
    EXPECT_EQ(sequence.size, COUNT);
    EXPECT_EQ(info_sequence.size, COUNT);
    for (size_t i = 0; i < COUNT; ++i) {
        EXPECT_EQ(messages[i].int64_value, static_cast<int64_t>(i));
    }

    ASSERT_RMW_OK(rmw_message_sequence_fini(&sequence));
    ASSERT_RMW_OK(rmw_message_info_sequence_fini(&info_sequence));
}

TEST_F(RmwPublishSubscribeTest, take_with_info_populates_message_info_non_self_contained) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    rcutils_time_point_value_t before_publish{0};
    ASSERT_EQ(rcutils_system_time_now(&before_publish), RCUTILS_RET_OK);

    auto send_payload = Strings{};
    send_payload.string_value = "GloryToHypnoToad";
    ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));

    void* recv_payload = malloc(sizeof(Strings));
    new (recv_payload) Strings{};
    bool taken{false};
    rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
    ASSERT_RMW_OK(rmw_take_with_info(subscription, recv_payload, &taken, &message_info, nullptr));
    ASSERT_TRUE(taken);

    rcutils_time_point_value_t after_take{0};
    ASSERT_EQ(rcutils_system_time_now(&after_take), RCUTILS_RET_OK);

    // Message info is carried in the user header for serialized (non-self-contained) payloads too.
    EXPECT_GE(message_info.source_timestamp, before_publish);
    EXPECT_LE(message_info.source_timestamp, message_info.received_timestamp);
    EXPECT_LE(message_info.received_timestamp, after_take);
    EXPECT_EQ(message_info.publication_sequence_number, 0u);
    EXPECT_FALSE(message_info.from_intra_process);

    ASSERT_EQ(*reinterpret_cast<Strings*>(recv_payload), send_payload);

    free(recv_payload);
}


TEST_F(RmwPublishSubscribeTest, take_with_info_publication_sequence_number_increments) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    constexpr uint64_t NUM_MESSAGES = 3;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    for (uint64_t i = 0; i < NUM_MESSAGES; ++i) {
        auto send_payload = Defaults{};
        ASSERT_RMW_OK(rmw_publish(publisher, &send_payload, nullptr));
    }

    void* recv_payload = malloc(sizeof(Defaults));
    for (uint64_t i = 0; i < NUM_MESSAGES; ++i) {
        bool taken{false};
        rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
        ASSERT_RMW_OK(rmw_take_with_info(subscription, recv_payload, &taken, &message_info, nullptr));
        ASSERT_TRUE(taken);
        EXPECT_EQ(message_info.publication_sequence_number, i);
    }

    free(recv_payload);
}

// ---------------------------------------------------------------------------
// Topic limits
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, more_than_sixteen_publishers_share_a_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    for (int i = 0; i < 20; ++i) {
        ASSERT_NE(create_default_publisher<Defaults>(topic), nullptr) << "publisher " << i + 1;
    }
}

TEST_F(RmwPublishSubscribeTest, more_than_sixteen_subscriptions_share_a_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    for (int i = 0; i < 20; ++i) {
        ASSERT_NE(create_default_subscriber<Defaults>(topic), nullptr) << "subscription " << i + 1;
    }
}

TEST_F(RmwPublishSubscribeTest, endpoints_join_an_event_service_created_by_another_iceoryx2_application) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service =
        native_node.service_builder(iox2::ServiceName::create(rmw::iox2::names::topic(topic.c_str()).c_str()).value())
            .event()
            .max_nodes(2)
            .max_notifiers(1)
            .max_listeners(1)
            .create();
    ASSERT_TRUE(native_service.has_value());

    ASSERT_NE(create_default_publisher<Defaults>(topic), nullptr);
    ASSERT_NE(create_default_subscriber<Defaults>(topic), nullptr);
    EXPECT_EQ(native_service->dynamic_config().number_of_notifiers(), 1U);
    EXPECT_EQ(native_service->dynamic_config().number_of_listeners(), 1U);
}

// ---------------------------------------------------------------------------
// Strict matching
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, reports_reliability_mismatch_when_subscriber_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();

    auto pub_profile = rmw_qos_profile_default;
    pub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    ASSERT_NE(create_publisher<Defaults>(topic, pub_profile), nullptr);

    auto sub_profile = rmw_qos_profile_default;
    sub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    EXPECT_EQ(create_subscriber<Defaults>(topic, sub_profile), nullptr);

    ASSERT_TRUE(rcutils_error_is_set());
    const std::string error{rcutils_get_error_string().str};
    EXPECT_NE(error.find("reliability"), std::string::npos);
    EXPECT_NE(error.find("RMW_IOX2_QOS_MATCHING=adoptive"), std::string::npos);
}

TEST_F(RmwPublishSubscribeTest, reports_reliability_mismatch_when_publisher_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();

    auto sub_profile = rmw_qos_profile_default;
    sub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    ASSERT_NE(create_subscriber<Defaults>(topic, sub_profile), nullptr);

    auto pub_profile = rmw_qos_profile_default;
    pub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    EXPECT_EQ(create_publisher<Defaults>(topic, pub_profile), nullptr);

    ASSERT_TRUE(rcutils_error_is_set());
    const std::string error{rcutils_get_error_string().str};
    EXPECT_NE(error.find("reliability"), std::string::npos);
}

TEST_F(RmwPublishSubscribeTest, reports_durability_mismatch_when_publisher_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();

    auto sub_profile = rmw_qos_profile_default;
    sub_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    ASSERT_NE(create_subscriber<Defaults>(topic, sub_profile), nullptr);

    auto pub_profile = rmw_qos_profile_default;
    pub_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    EXPECT_EQ(create_publisher<Defaults>(topic, pub_profile), nullptr);

    ASSERT_TRUE(rcutils_error_is_set());
    const std::string error{rcutils_get_error_string().str};
    EXPECT_NE(error.find("durability"), std::string::npos);
}

TEST_F(RmwPublishSubscribeTest, reports_durability_mismatch_when_subscriber_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();

    auto pub_profile = rmw_qos_profile_default;
    pub_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    ASSERT_NE(create_publisher<Defaults>(topic, pub_profile), nullptr);

    auto sub_profile = rmw_qos_profile_default;
    sub_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    EXPECT_EQ(create_subscriber<Defaults>(topic, sub_profile), nullptr);

    ASSERT_TRUE(rcutils_error_is_set());
    const std::string error{rcutils_get_error_string().str};
    EXPECT_NE(error.find("durability"), std::string::npos);
}

TEST_F(RmwPublishSubscribeTest, reports_depth_mismatch_when_subscriber_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();

    auto pub_profile = rmw_qos_profile_default;
    pub_profile.depth = 10;
    ASSERT_NE(create_publisher<Defaults>(topic, pub_profile), nullptr);

    auto sub_profile = rmw_qos_profile_default;
    sub_profile.depth = 42;
    EXPECT_EQ(create_subscriber<Defaults>(topic, sub_profile), nullptr);

    ASSERT_TRUE(rcutils_error_is_set());
    const std::string error{rcutils_get_error_string().str};
    EXPECT_NE(error.find("history"), std::string::npos);
}

// ---------------------------------------------------------------------------
// Adoptive matching
// ---------------------------------------------------------------------------

class RmwPublishSubscribeQosEnvTest : public TestBase
{
protected:
    void SetUp() override {
        unset_environment("RMW_IOX2_QOS_MATCHING");
        unset_environment("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_NODES_PER_SERVICE");
    }

    void TearDown() override {
        if (m_initialized) {
            cleanup();
        }
        restore_environment();
        print_rmw_errors();
    }

    void initialize_with_env() {
        initialize();
        m_initialized = true;
    }

private:
    bool m_initialized{false};
};

TEST_F(RmwPublishSubscribeQosEnvTest, adoptive_subscriber_inherits_publisher_reliability) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "adoptive", 1);
    initialize_with_env();

    auto topic = create_test_topic();

    auto pub_profile = rmw_qos_profile_default;
    pub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    ASSERT_NE(create_publisher<Defaults>(topic, pub_profile), nullptr);

    auto* sub = create_default_subscriber<Defaults>(topic);
    ASSERT_NE(sub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_subscription_get_actual_qos(sub, &actual));
    EXPECT_EQ(actual.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(RmwPublishSubscribeQosEnvTest, adoptive_publisher_inherits_subscriber_durability) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "adoptive", 1);
    initialize_with_env();

    auto topic = create_test_topic();

    auto sub_profile = rmw_qos_profile_default;
    sub_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    ASSERT_NE(create_subscriber<Defaults>(topic, sub_profile), nullptr);

    auto* pub = create_default_publisher<Defaults>(topic);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

} // namespace
