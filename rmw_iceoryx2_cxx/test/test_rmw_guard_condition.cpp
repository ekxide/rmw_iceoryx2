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
#include <thread>

namespace
{

using namespace rmw::iox2::testing;

class RmwGuardConditionTest : public TestBase
{
protected:
    static auto wait_for(rmw_guard_condition_t* guard_condition, rmw_wait_set_t* wait_set) -> bool {
        void* conditions[] = {guard_condition->data};
        rmw_guard_conditions_t guard_conditions{1, conditions};
        rmw_time_t timeout{1, 0};
        auto result = rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, wait_set, &timeout);
        return result == RMW_RET_OK && conditions[0] != nullptr;
    }

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

TEST_F(RmwGuardConditionTest, wake_up_once_per_trigger_between_threads) {
    constexpr size_t ROUND_TRIPS = 1000;

    auto ping = rmw_create_guard_condition(test_context());
    ASSERT_NE(ping, nullptr);
    auto pong = rmw_create_guard_condition(test_context());
    ASSERT_NE(pong, nullptr);
    auto ping_wait_set = rmw_create_wait_set(test_context(), 1);
    ASSERT_NE(ping_wait_set, nullptr);
    auto pong_wait_set = rmw_create_wait_set(test_context(), 1);
    ASSERT_NE(pong_wait_set, nullptr);

    size_t responded = 0;
    std::thread responder([&] {
        while (responded < ROUND_TRIPS && wait_for(ping, pong_wait_set)) {
            ++responded;
            EXPECT_RMW_OK(rmw_trigger_guard_condition(pong));
        }
    });
    size_t completed = 0;
    while (completed < ROUND_TRIPS && rmw_trigger_guard_condition(ping) == RMW_RET_OK
           && wait_for(pong, ping_wait_set)) {
        ++completed;
    }
    responder.join();

    EXPECT_EQ(completed, ROUND_TRIPS);
    EXPECT_EQ(responded, ROUND_TRIPS);

    EXPECT_RMW_OK(rmw_destroy_wait_set(pong_wait_set));
    EXPECT_RMW_OK(rmw_destroy_wait_set(ping_wait_set));
    EXPECT_RMW_OK(rmw_destroy_guard_condition(pong));
    EXPECT_RMW_OK(rmw_destroy_guard_condition(ping));
}

} // namespace
