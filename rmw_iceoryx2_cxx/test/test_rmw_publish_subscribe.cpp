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
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/arrays.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/basic_types.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/bounded_sequences.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/empty.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/padded.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/unbounded_sequences.hpp"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

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

    template <typename T>
    void expect_serialized_round_trip(const T& input) {
        auto* publisher = create_default_publisher<T>(create_test_topic());
        ASSERT_NE(publisher, nullptr);
        auto* subscription = create_default_subscriber<T>(create_test_topic());
        ASSERT_NE(subscription, nullptr);
        rmw_serialized_message_t expected{};
        ASSERT_RMW_OK(rmw_serialized_message_init(&expected, 0, &test_allocator()));
        ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<T>(), &expected));

        ASSERT_RMW_OK(rmw_publish(publisher, &input, nullptr));
        rmw_serialized_message_t serialized{};
        ASSERT_RMW_OK(rmw_serialized_message_init(&serialized, 0, &test_allocator()));
        bool taken{false};
        ASSERT_RMW_OK(rmw_take_serialized_message(subscription, &serialized, &taken, nullptr));
        ASSERT_TRUE(taken);
        T deserialized{};
        ASSERT_RMW_OK(rmw_deserialize(&serialized, test_type_support<T>(), &deserialized));
        EXPECT_EQ(deserialized, input);

        ASSERT_RMW_OK(rmw_publish_serialized_message(publisher, &expected, nullptr));
        T output{};
        ASSERT_RMW_OK(rmw_take(subscription, &output, &taken, nullptr));
        ASSERT_TRUE(taken);
        EXPECT_EQ(output, input);

        ASSERT_RMW_OK(rmw_serialized_message_fini(&expected));
        ASSERT_RMW_OK(rmw_serialized_message_fini(&serialized));
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

TEST_F(RmwPublishSubscribeTest, take_serialized_message_reports_its_length) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    Strings input{};
    input.string_value = "GloryToHypnoToad";
    rmw_serialized_message_t input_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&input_serialized_msg, sizeof(Strings), &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &input_serialized_msg));
    ASSERT_RMW_OK(rmw_publish_serialized_message(publisher, &input_serialized_msg, nullptr));

    rmw_serialized_message_t output_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&output_serialized_msg, 0, &test_allocator()));
    bool taken{false};
    ASSERT_RMW_OK(rmw_take_serialized_message(subscription, &output_serialized_msg, &taken, nullptr));
    ASSERT_TRUE(taken);
    EXPECT_EQ(output_serialized_msg.buffer_length, input_serialized_msg.buffer_length);

    ASSERT_RMW_OK(rmw_serialized_message_fini(&input_serialized_msg));
    ASSERT_RMW_OK(rmw_serialized_message_fini(&output_serialized_msg));
}

TEST_F(RmwPublishSubscribeTest, serialized_take_and_publish_convert_basic_types) {
    rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes input{};
    input.bool_value = true;
    input.char_value = 'x';
    input.int16_value = -16;
    input.float64_value = 2.5;
    input.uint64_value = 64;
    expect_serialized_round_trip(input);
}

TEST_F(RmwPublishSubscribeTest, serialized_take_and_publish_convert_empty) {
    expect_serialized_round_trip(rmw_iceoryx2_cxx_test_msgs::msg::Empty{});
}

TEST_F(RmwPublishSubscribeTest, serialized_take_and_publish_convert_arrays) {
    rmw_iceoryx2_cxx_test_msgs::msg::Arrays input{};
    input.int32_values = {1, -2, 3};
    input.string_values = {"GloryTo", "", "HypnoToad"};
    input.basic_types_values[1].float32_value = 1.5F;
    expect_serialized_round_trip(input);
}

TEST_F(RmwPublishSubscribeTest, serialized_take_and_publish_convert_unbounded_sequences) {
    rmw_iceoryx2_cxx_test_msgs::msg::UnboundedSequences input{};
    input.uint8_values = {1, 2, 3, 4, 5};
    input.string_values = {"GloryToHypnoToad"};
    input.basic_types_values.resize(2);
    input.basic_types_values[1].int64_value = -64;
    expect_serialized_round_trip(input);
}

TEST_F(RmwPublishSubscribeTest, serialized_take_and_publish_convert_padded_types) {
    rmw_iceoryx2_cxx_test_msgs::msg::Padded input{};
    input.first = 1;
    input.basic_types_value.bool_value = true;
    input.basic_types_value.float64_value = 2.5;
    input.last = 3;
    expect_serialized_round_trip(input);
}

TEST_F(RmwPublishSubscribeTest, take_serialized_message_with_info_populates_message_info) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    auto* publisher = create_default_publisher<Strings>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Strings>(create_test_topic());
    ASSERT_NE(subscription, nullptr);

    Strings input{};
    input.string_value = "GloryToHypnoToad";
    ASSERT_RMW_OK(rmw_publish(publisher, &input, nullptr));

    rmw_serialized_message_t output_serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&output_serialized_msg, 0, &test_allocator()));
    rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
    bool taken{false};
    ASSERT_RMW_OK(
        rmw_take_serialized_message_with_info(subscription, &output_serialized_msg, &taken, &message_info, nullptr));
    ASSERT_TRUE(taken);
    EXPECT_NE(message_info.source_timestamp, 0);
    rmw_gid_t publisher_gid{};
    ASSERT_RMW_OK(rmw_get_gid_for_publisher(publisher, &publisher_gid));
    bool same_publisher{false};
    ASSERT_RMW_OK(rmw_compare_gids_equal(&publisher_gid, &message_info.publisher_gid, &same_publisher));
    EXPECT_TRUE(same_publisher);

    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&output_serialized_msg, test_type_support<Strings>(), &output));
    EXPECT_EQ(input, output);
    ASSERT_RMW_OK(rmw_serialized_message_fini(&output_serialized_msg));
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
    EXPECT_EQ(message_info.publication_sequence_number, 1u);
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
    EXPECT_EQ(message_info.publication_sequence_number, 1u);
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
        // Numbered from 1, like the requests of a client.
        EXPECT_EQ(message_info.publication_sequence_number, i + 1);
    }

    free(recv_payload);
}

// ---------------------------------------------------------------------------
// Concurrent calls
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, a_late_transient_local_subscription_receives_the_history_of_a_quiet_publisher) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    constexpr int64_t PUBLISHED_VALUE = 4711;

    auto qos = rmw_qos_profile_default;
    qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto* publisher = create_publisher<Defaults>(create_test_topic(), qos);
    ASSERT_NE(publisher, nullptr);
    Defaults input{};
    input.int64_value = PUBLISHED_VALUE;
    ASSERT_RMW_OK(rmw_publish(publisher, &input, nullptr));

    auto* subscription = create_subscriber<Defaults>(create_test_topic(), qos);
    ASSERT_NE(subscription, nullptr);

    auto* wait_set = rmw_create_wait_set(test_context(), 1);
    ASSERT_NE(wait_set, nullptr);
    std::array<void*, 1> ready{subscription->data};
    rmw_subscriptions_t subscriptions{ready.size(), ready.data()};
    ASSERT_RMW_OK(rmw_wait(&subscriptions, nullptr, nullptr, nullptr, nullptr, wait_set, nullptr));
    ASSERT_RMW_OK(rmw_destroy_wait_set(wait_set));

    Defaults output{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take(subscription, &output, &taken, nullptr));
    ASSERT_TRUE(taken);
    EXPECT_EQ(output.int64_value, PUBLISHED_VALUE);
}

TEST_F(RmwPublishSubscribeTest, concurrent_loans_of_a_publisher_are_all_returned) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    constexpr int NUM_THREADS = 2;
    constexpr int NUM_LOANS = 5000;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);

    std::atomic<int> failures{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < NUM_THREADS; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < NUM_LOANS; ++i) {
                void* loan = nullptr;
                if (rmw_borrow_loaned_message(publisher, test_type_support<Defaults>(), &loan) != RMW_RET_OK
                    || rmw_return_loaned_message_from_publisher(publisher, loan) != RMW_RET_OK) {
                    ++failures;
                }
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }

    EXPECT_EQ(failures.load(), 0);
}

TEST_F(RmwPublishSubscribeTest, concurrent_publishes_number_every_message) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    constexpr int NUM_THREADS = 4;
    constexpr int NUM_MESSAGES = 5000;

    auto qos = rmw_qos_profile_default;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto* publisher = create_publisher<Defaults>(create_test_topic(), qos);
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_subscriber<Defaults>(create_test_topic(), qos);
    ASSERT_NE(subscription, nullptr);

    std::vector<std::thread> threads;
    for (int t = 0; t < NUM_THREADS; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < NUM_MESSAGES; ++i) {
                auto payload = Defaults{};
                EXPECT_EQ(rmw_publish(publisher, &payload, nullptr), RMW_RET_OK);
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }

    uint64_t last_sequence_number = 0;
    auto payload = Defaults{};
    bool taken{true};
    while (taken) {
        rmw_message_info_t message_info = rmw_get_zero_initialized_message_info();
        ASSERT_RMW_OK(rmw_take_with_info(subscription, &payload, &taken, &message_info, nullptr));
        if (taken) {
            last_sequence_number = std::max(last_sequence_number, message_info.publication_sequence_number);
        }
    }

    EXPECT_EQ(last_sequence_number, static_cast<uint64_t>(NUM_THREADS) * NUM_MESSAGES);
}

TEST_F(RmwPublishSubscribeTest, concurrent_loaned_takes_of_a_subscription_are_all_returned) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    constexpr int NUM_THREADS = 2;
    constexpr int NUM_TAKES = 5000;

    auto qos = rmw_qos_profile_default;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto* publisher = create_publisher<Defaults>(create_test_topic(), qos);
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_subscriber<Defaults>(create_test_topic(), qos);
    ASSERT_NE(subscription, nullptr);

    std::atomic<bool> done{false};
    std::thread publishing_thread([&] {
        while (!done) {
            auto payload = Defaults{};
            EXPECT_RMW_OK(rmw_publish(publisher, &payload, nullptr));
        }
    });

    std::atomic<int> failures{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < NUM_THREADS; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < NUM_TAKES; ++i) {
                void* loan = nullptr;
                bool taken{false};
                if (rmw_take_loaned_message(subscription, &loan, &taken, nullptr) != RMW_RET_OK) {
                    ++failures;
                } else if (taken && rmw_return_loaned_message_from_subscription(subscription, loan) != RMW_RET_OK) {
                    ++failures;
                }
            }
        });
    }
    for (auto& thread : threads) {
        thread.join();
    }
    done = true;
    publishing_thread.join();

    EXPECT_EQ(failures.load(), 0);
}

// ---------------------------------------------------------------------------
// Topic limits
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, creates_publishers_up_to_the_topic_limit) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    for (size_t i = 0; i < rmw::iox2::DEFAULT_MAX_PUBLISHERS_PER_TOPIC; ++i) {
        ASSERT_NE(create_default_publisher<Defaults>(topic), nullptr) << "publisher " << i + 1;
    }
}

TEST_F(RmwPublishSubscribeTest, creates_subscriptions_up_to_the_topic_limit) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    for (size_t i = 0; i < rmw::iox2::DEFAULT_MAX_SUBSCRIBERS_PER_TOPIC; ++i) {
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

TEST_F(RmwPublishSubscribeTest, a_message_is_published_when_waking_up_the_subscriptions_fails) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service =
        native_node.service_builder(iox2::ServiceName::create(rmw::iox2::names::topic(topic.c_str()).c_str()).value())
            .event()
            .deadline(iox2::bb::Duration::from_nanos(1))
            .create();
    ASSERT_TRUE(native_service.has_value());
    auto* publisher = create_default_publisher<Defaults>(topic);
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(topic);
    ASSERT_NE(subscription, nullptr);

    auto message = Defaults{};
    message.int32_value = 1;
    ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    void* publisher_loan = nullptr;
    ASSERT_RMW_OK(rmw_borrow_loaned_message(publisher, test_type_support<Defaults>(), &publisher_loan));
    new (publisher_loan) Defaults{};
    static_cast<Defaults*>(publisher_loan)->int32_value = 2;
    ASSERT_RMW_OK(rmw_publish_loaned_message(publisher, publisher_loan, nullptr));

    auto received = Defaults{};
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &received, &taken, nullptr));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received.int32_value, 1);
    ASSERT_RMW_OK(rmw_take(subscription, &received, &taken, nullptr));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received.int32_value, 2);
}

// ---------------------------------------------------------------------------
// Topic sizing
// ---------------------------------------------------------------------------

class RmwPublishSubscribeQosEnvTest : public TestBase
{
protected:
    void SetUp() override {
        unset_environment("RMW_IOX2_QOS_MATCHING");
        unset_environment("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_NODES_PER_SERVICE");
        unset_environment("RMW_IOX2_MAX_DEPTH_PER_TOPIC");
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

TEST_F(RmwPublishSubscribeTest, volatile_subscription_gets_no_history) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    auto* publisher = create_default_publisher<Defaults>(topic);
    ASSERT_NE(publisher, nullptr);
    const Defaults message{};
    ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));

    auto* subscription = create_default_subscriber<Defaults>(topic);
    ASSERT_NE(subscription, nullptr);
    ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    int received_count = 0;
    bool taken{true};
    while (taken) {
        Defaults received{};
        ASSERT_RMW_OK(rmw_take(subscription, &received, &taken, nullptr));
        received_count += taken;
    }
    EXPECT_EQ(received_count, 1);
}

TEST_F(RmwPublishSubscribeQosEnvTest, depth_above_the_maximum_is_capped) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_MAX_DEPTH_PER_TOPIC", "5", 1);
    initialize_with_env();

    auto topic = create_test_topic();
    auto* publisher = create_default_publisher<Defaults>(topic);
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(topic);
    ASSERT_NE(subscription, nullptr);

    const Defaults message{};
    for (int i = 0; i < 10; ++i) {
        ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    }
    int received_count = 0;
    bool taken{true};
    while (taken) {
        Defaults received{};
        ASSERT_RMW_OK(rmw_take(subscription, &received, &taken, nullptr));
        received_count += taken;
    }
    EXPECT_EQ(received_count, 5);
}

// ---------------------------------------------------------------------------
// ROS matching
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, subscription_with_a_larger_depth_joins_a_publisher) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    auto pub_profile = rmw_qos_profile_default;
    pub_profile.depth = 7;
    auto* publisher = create_publisher<Defaults>(topic, pub_profile);
    ASSERT_NE(publisher, nullptr);
    auto* subscription = create_default_subscriber<Defaults>(topic);
    ASSERT_NE(subscription, nullptr);

    const Defaults message{};
    ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    Defaults received{};
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &received, &taken, nullptr));
    EXPECT_TRUE(taken);
}

TEST_F(RmwPublishSubscribeTest, publisher_with_a_smaller_depth_joins_a_subscription) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    auto* subscription = create_default_subscriber<Defaults>(topic);
    ASSERT_NE(subscription, nullptr);
    auto pub_profile = rmw_qos_profile_default;
    pub_profile.depth = 7;
    auto* publisher = create_publisher<Defaults>(topic, pub_profile);
    ASSERT_NE(publisher, nullptr);

    const Defaults message{};
    ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    Defaults received{};
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &received, &taken, nullptr));
    EXPECT_TRUE(taken);
}

TEST_F(RmwPublishSubscribeTest, best_effort_subscription_joins_a_reliable_publisher) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic();
    ASSERT_NE(create_default_publisher<Defaults>(topic), nullptr);
    auto sub_profile = rmw_qos_profile_default;
    sub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    auto* subscription = create_subscriber<Defaults>(topic, sub_profile);
    ASSERT_NE(subscription, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_subscription_get_actual_qos(subscription, &actual));
    EXPECT_EQ(actual.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(RmwPublishSubscribeTest, publishers_with_different_depths_created_at_once_join_the_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    constexpr size_t TOPICS = 40U;
    constexpr size_t THREADS = 2U;
    constexpr size_t FIRST_DEPTH = 3U;
    constexpr size_t SECOND_DEPTH = 7U;

    auto* other_node = create_default_node("Other");
    ASSERT_NE(other_node, nullptr);
    std::vector<std::string> topics;
    for (size_t i = 0; i < TOPICS; ++i) {
        topics.push_back(create_test_topic("/Concurrent" + std::to_string(i)));
    }

    std::atomic<size_t> arrived{0};
    auto create = [&](rmw_node_t* node, size_t depth) {
        auto qos = rmw_qos_profile_default;
        qos.depth = depth;
        auto options = rmw_get_default_publisher_options();
        std::vector<rmw_publisher_t*> publishers;
        for (size_t i = 0; i < TOPICS; ++i) {
            arrived.fetch_add(1);
            while (arrived.load() < THREADS * (i + 1)) {
            }
            publishers.push_back(
                rmw_create_publisher(node, test_type_support<Defaults>(), topics[i].c_str(), &qos, &options));
        }
        return publishers;
    };

    std::vector<rmw_publisher_t*> second;
    std::thread other{[&] { second = create(other_node, SECOND_DEPTH); }};
    auto first = create(test_node(), FIRST_DEPTH);
    other.join();

    EXPECT_EQ(std::count(first.begin(), first.end(), nullptr), 0);
    EXPECT_EQ(std::count(second.begin(), second.end(), nullptr), 0);
    for (auto* publisher : first) {
        if (publisher != nullptr) {
            EXPECT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
        }
    }
    for (auto* publisher : second) {
        if (publisher != nullptr) {
            EXPECT_RMW_OK(rmw_destroy_publisher(other_node, publisher));
        }
    }
}

// ---------------------------------------------------------------------------
// Strict matching
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeQosEnvTest, reports_reliability_mismatch_when_subscriber_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);
    initialize_with_env();

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

TEST_F(RmwPublishSubscribeQosEnvTest, reports_reliability_mismatch_when_publisher_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);
    initialize_with_env();

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

TEST_F(RmwPublishSubscribeQosEnvTest, reports_durability_mismatch_when_publisher_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);
    initialize_with_env();

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

TEST_F(RmwPublishSubscribeQosEnvTest, reports_durability_mismatch_when_subscriber_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);
    initialize_with_env();

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

TEST_F(RmwPublishSubscribeQosEnvTest, reports_depth_mismatch_when_subscriber_joins) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);
    initialize_with_env();

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

// ---------------------------------------------------------------------------
// Local publications
// ---------------------------------------------------------------------------

TEST_F(RmwPublishSubscribeTest, a_subscription_ignoring_local_publications_does_not_take_them) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    auto options = rmw_get_default_subscription_options();
    options.ignore_local_publications = true;
    auto qos = rmw_qos_profile_default;
    auto* subscription = rmw_create_subscription(
        test_node(), test_type_support<Defaults>(), create_test_topic().c_str(), &qos, &options);
    ASSERT_NE(subscription, nullptr);

    auto message = Defaults{};
    ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    bool taken{true};
    ASSERT_RMW_OK(rmw_take(subscription, &message, &taken, nullptr));
    EXPECT_FALSE(taken);

    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
}

TEST_F(RmwPublishSubscribeTest, a_subscription_ignoring_local_publications_takes_those_of_other_contexts) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    auto other_context = rmw_get_zero_initialized_context();
    ASSERT_RMW_OK(rmw_init(&init_options, &other_context));
    auto* other_node = rmw_create_node(&other_context, "OtherNode", "/");
    ASSERT_NE(other_node, nullptr);
    auto qos = rmw_qos_profile_default;
    auto publisher_options = rmw_get_default_publisher_options();
    auto* other_publisher = rmw_create_publisher(
        other_node, test_type_support<Defaults>(), create_test_topic().c_str(), &qos, &publisher_options);
    ASSERT_NE(other_publisher, nullptr);
    auto* local_publisher = create_default_publisher<Defaults>(create_test_topic());
    ASSERT_NE(local_publisher, nullptr);
    auto options = rmw_get_default_subscription_options();
    options.ignore_local_publications = true;
    auto* subscription = rmw_create_subscription(
        test_node(), test_type_support<Defaults>(), create_test_topic().c_str(), &qos, &options);
    ASSERT_NE(subscription, nullptr);

    auto local_message = Defaults{};
    local_message.int32_value = 1;
    ASSERT_RMW_OK(rmw_publish(local_publisher, &local_message, nullptr));
    auto other_message = Defaults{};
    other_message.int32_value = 2;
    ASSERT_RMW_OK(rmw_publish(other_publisher, &other_message, nullptr));

    auto message = Defaults{};
    bool taken{false};
    ASSERT_RMW_OK(rmw_take(subscription, &message, &taken, nullptr));
    EXPECT_TRUE(taken);
    EXPECT_EQ(message.int32_value, 2);
    ASSERT_RMW_OK(rmw_take(subscription, &message, &taken, nullptr));
    EXPECT_FALSE(taken);

    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
    ASSERT_RMW_OK(rmw_destroy_publisher(other_node, other_publisher));
    ASSERT_RMW_OK(rmw_destroy_node(other_node));
    ASSERT_RMW_OK(rmw_shutdown(&other_context));
    ASSERT_RMW_OK(rmw_context_fini(&other_context));
    ASSERT_RMW_OK(rmw_init_options_fini(&init_options));
}

} // namespace
