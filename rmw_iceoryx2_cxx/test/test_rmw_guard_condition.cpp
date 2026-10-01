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
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <filesystem>
#include <iterator>

namespace
{

using namespace rmw::iox2::testing;

class RmwGuardConditionTest : public TestBase
{
protected:
    void SetUp() override {
        initialize_test_context();
    }

    void TearDown() override {
        cleanup_test_context();
        print_rmw_errors();
    }
};

TEST_F(RmwGuardConditionTest, create_and_destroy) {
    auto guard_condition = rmw_create_guard_condition(test_context());

    RMW_ASSERT_NE(guard_condition, nullptr);
    RMW_ASSERT_NE(guard_condition->context, nullptr);
    RMW_ASSERT_NE(guard_condition->implementation_identifier, nullptr);
    RMW_ASSERT_NE(guard_condition->data, nullptr);

    ASSERT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

TEST_F(RmwGuardConditionTest, trigger) {
    auto guard_condition = rmw_create_guard_condition(test_context());
    ASSERT_NE(guard_condition, nullptr);
    auto wait_set = rmw_create_wait_set(test_context(), 1);
    ASSERT_NE(wait_set, nullptr);

    EXPECT_RMW_OK(rmw_trigger_guard_condition(guard_condition));
    void* conditions[] = {guard_condition->data};
    rmw_guard_conditions_t guard_conditions{1, conditions};
    rmw_time_t timeout{0, 500000};
    EXPECT_RMW_OK(rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, wait_set, &timeout));
    EXPECT_NE(conditions[0], nullptr);

    EXPECT_RMW_OK(rmw_destroy_wait_set(wait_set));
    EXPECT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

TEST_F(RmwGuardConditionTest, trigger_more_guard_conditions_than_event_ids) {
    constexpr size_t GUARD_CONDITION_COUNT = 300;

    for (size_t i = 0; i < GUARD_CONDITION_COUNT; ++i) {
        auto guard_condition = rmw_create_guard_condition(test_context());
        ASSERT_NE(guard_condition, nullptr);
        ASSERT_RMW_OK(rmw_trigger_guard_condition(guard_condition));
        ASSERT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
    }
}

TEST_F(RmwGuardConditionTest, destroying_a_guard_condition_closes_its_file_descriptors) {
    constexpr size_t GUARD_CONDITION_COUNT = 100;
    if (!std::filesystem::exists("/proc/self/fd")) {
        GTEST_SKIP() << "needs /proc/self/fd to count file descriptors";
    }
    auto open_file_descriptors = [] {
        return std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
                             std::filesystem::directory_iterator{});
    };

    const auto before = open_file_descriptors();
    for (size_t i = 0; i < GUARD_CONDITION_COUNT; ++i) {
        auto guard_condition = rmw_create_guard_condition(test_context());
        ASSERT_NE(guard_condition, nullptr);
        ASSERT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
    }
    EXPECT_EQ(open_file_descriptors(), before);
}

} // namespace
