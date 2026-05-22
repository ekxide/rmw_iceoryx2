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
using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

// ---------------------------------------------------------------------------
// rmw_publisher_get_actual_qos / rmw_subscription_get_actual_qos round-trips
// ---------------------------------------------------------------------------

class RmwQosRoundTripTest : public TestBase
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

TEST_F(RmwQosRoundTripTest, properly_reports_publisher_reliability_best_effort) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(RmwQosRoundTripTest, properly_reports_publisher_durability_transient_local) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

TEST_F(RmwQosRoundTripTest, properly_reports_publisher_history_depth) {
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

TEST_F(RmwQosRoundTripTest, properly_reports_publisher_deadline) {
    auto profile = rmw_qos_profile_default;
    profile.deadline = {5U, 250000000U};

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.deadline.sec, 5U);
    EXPECT_EQ(actual.deadline.nsec, 250000000U);
}

TEST_F(RmwQosRoundTripTest, properly_reports_publisher_lifespan) {
    auto profile = rmw_qos_profile_default;
    profile.lifespan = {2U, 0U};

    auto* pub = create_publisher<Defaults>(create_test_topic(), profile);
    ASSERT_NE(pub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_publisher_get_actual_qos(pub, &actual));
    EXPECT_EQ(actual.lifespan.sec, 2U);
    EXPECT_EQ(actual.lifespan.nsec, 0U);
}

TEST_F(RmwQosRoundTripTest, properly_reports_publisher_liveliness_manual_by_topic) {
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

TEST_F(RmwQosRoundTripTest, properly_reports_subscriber_reliability_best_effort) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    auto* sub = create_subscriber<Defaults>(create_test_topic(), profile);
    ASSERT_NE(sub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_subscription_get_actual_qos(sub, &actual));
    EXPECT_EQ(actual.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
}

TEST_F(RmwQosRoundTripTest, properly_reports_subscriber_durability_transient_local) {
    auto profile = rmw_qos_profile_default;
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

    auto* sub = create_subscriber<Defaults>(create_test_topic(), profile);
    ASSERT_NE(sub, nullptr);

    rmw_qos_profile_t actual = {};
    ASSERT_RMW_OK(rmw_subscription_get_actual_qos(sub, &actual));
    EXPECT_EQ(actual.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
}

// ---------------------------------------------------------------------------
// rmw_qos_profile_check_compatible
// ---------------------------------------------------------------------------

class RmwQosCheckCompatibleTest : public ::testing::Test
{
};

TEST_F(RmwQosCheckCompatibleTest, accepts_identical_concrete_profiles) {
    // All fields explicitly concrete so no policy triggers a "cannot determine"
    // warning in the compatibility check.
    rmw_qos_profile_t profile = rmw_qos_profile_default;
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    profile.depth = 10;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    profile.deadline = {0U, 0U};
    profile.lifespan = {0U, 0U};
    profile.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    profile.liveliness_lease_duration = {0U, 0U};
    profile.avoid_ros_namespace_conventions = false;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(profile, profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_OK);
}

TEST_F(RmwQosCheckCompatibleTest, rejects_reliability_mismatch) {
    auto pub_profile = rmw_qos_profile_default;
    pub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    auto sub_profile = rmw_qos_profile_default;
    sub_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_OK;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(pub_profile, sub_profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_NE(std::string{reason}.find("reliability"), std::string::npos);
}

TEST_F(RmwQosCheckCompatibleTest, rejects_durability_mismatch) {
    auto pub_profile = rmw_qos_profile_default;
    pub_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    auto sub_profile = rmw_qos_profile_default;
    sub_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_OK;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(pub_profile, sub_profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_NE(std::string{reason}.find("durability"), std::string::npos);
}

TEST_F(RmwQosCheckCompatibleTest, rejects_depth_mismatch) {
    auto pub_profile = rmw_qos_profile_default;
    pub_profile.depth = 10;
    auto sub_profile = rmw_qos_profile_default;
    sub_profile.depth = 20;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_OK;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(pub_profile, sub_profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_NE(std::string{reason}.find("depth"), std::string::npos);
}

TEST_F(RmwQosCheckCompatibleTest, rejects_keep_all_history) {
    auto profile = rmw_qos_profile_default;
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_OK;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(profile, profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_NE(std::string{reason}.find("KEEP_ALL"), std::string::npos);
}

TEST_F(RmwQosCheckCompatibleTest, warns_on_system_default_reliability) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_OK;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(profile, profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_WARNING);
}

TEST_F(RmwQosCheckCompatibleTest, rejects_null_compatibility_argument) {
    EXPECT_EQ(rmw_qos_profile_check_compatible(rmw_qos_profile_default, rmw_qos_profile_default, nullptr, nullptr, 0),
              RMW_RET_INVALID_ARGUMENT);
}

} // namespace
