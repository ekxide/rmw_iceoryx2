// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <string>

namespace
{

using namespace rmw::iox2::testing;

// ---------------------------------------------------------------------------
// profile_check_compatible
// ---------------------------------------------------------------------------

class RmwQosCheckCompatibleTest : public TestBase
{
protected:
    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(RmwQosCheckCompatibleTest, accepts_identical_concrete_profiles) {
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

TEST_F(RmwQosCheckCompatibleTest, accepts_system_default_reliability_on_both_sides) {
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;

    rmw_qos_compatibility_type_t compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    char reason[256] = {0};

    EXPECT_RMW_OK(rmw_qos_profile_check_compatible(profile, profile, &compatibility, reason, sizeof(reason)));
    EXPECT_EQ(compatibility, RMW_QOS_COMPATIBILITY_OK);
}

TEST_F(RmwQosCheckCompatibleTest, warns_on_unknown_reliability) {
    // UNKNOWN cannot be resolved, so the check cannot predict whether the
    // endpoints will connect.
    auto profile = rmw_qos_profile_default;
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;

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
