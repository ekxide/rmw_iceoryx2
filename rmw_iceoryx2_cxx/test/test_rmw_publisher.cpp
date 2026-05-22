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

#include <cstdlib>

namespace
{
using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

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

// ---------------------------------------------------------------------------
// Creation
// ---------------------------------------------------------------------------


TEST_F(RmwPublisherTest, create_and_destroy) {
    auto* publisher = create_default_publisher<Defaults>(create_test_topic());

    RMW_ASSERT_NE(publisher, nullptr);
    RMW_ASSERT_NE(publisher->implementation_identifier, nullptr);
    RMW_ASSERT_NE(publisher->data, nullptr);
    ASSERT_STREQ(publisher->topic_name, create_test_topic().c_str());
    RMW_ASSERT_TRUE(publisher->can_loan_messages);
}

// ---------------------------------------------------------------------------
// get_actual_qos
// ---------------------------------------------------------------------------

TEST_F(RmwPublisherTest, borrow_and_return) {
    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    void* loaned_message = nullptr;

    EXPECT_RMW_OK(rmw_borrow_loaned_message(publisher, test_type_support<Defaults>(), &loaned_message));
    EXPECT_RMW_OK(rmw_return_loaned_message_from_publisher(publisher, loaned_message));
}

TEST_F(RmwPublisherTest, properly_reports_reliability_best_effort) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(RmwPublisherTest, properly_reports_durability_transient_local) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

TEST_F(RmwPublisherTest, properly_reports_history_depth) {
    auto profile = rmw_qos_profile_default;
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    profile.depth = 42;

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    EXPECT_EQ(actual.depth, 42U);
}

TEST_F(RmwPublisherTest, properly_reports_deadline) {
    auto profile = rmw_qos_profile_default;
    profile.deadline = {5U, 250000000U};

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.deadline.sec, 5U);
    EXPECT_EQ(actual.deadline.nsec, 250000000U);
}

TEST_F(RmwPublisherTest, properly_reports_lifespan) {
    auto profile = rmw_qos_profile_default;
    profile.lifespan = {2U, 0U};

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.lifespan.sec, 2U);
    EXPECT_EQ(actual.lifespan.nsec, 0U);
}

TEST_F(RmwPublisherTest, properly_reports_liveliness_manual_by_topic) {
    auto profile = rmw_qos_profile_default;
    profile.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    profile.liveliness_lease_duration = {1U, 0U};

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.liveliness, RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    EXPECT_EQ(actual.liveliness_lease_duration.sec, 1U);
    EXPECT_EQ(actual.liveliness_lease_duration.nsec, 0U);
}

TEST_F(RmwPublisherTest, rejects_keep_all_history) {
    auto profile = rmw_qos_profile_default;
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    EXPECT_EQ(create_publisher<Defaults>(create_test_topic(), profile), nullptr);
}

// ---------------------------------------------------------------------------
// Environment configuration
// ---------------------------------------------------------------------------

class RmwPublisherEnvironmentConfigurationTest : public TestBase
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

TEST_F(RmwPublisherEnvironmentConfigurationTest, caps_publishers_at_configured_max) {
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "2", 1);
    initialize_with_env();

    auto topic = create_test_topic();
    EXPECT_NE(create_default_publisher<Defaults>(topic), nullptr);
    EXPECT_NE(create_default_publisher<Defaults>(topic), nullptr);
    EXPECT_EQ(create_default_publisher<Defaults>(topic), nullptr);
}

} // namespace
