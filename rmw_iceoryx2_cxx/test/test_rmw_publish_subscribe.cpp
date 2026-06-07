// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rcutils/error_handling.h"
#include "rcutils/time.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

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
