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

#include <thread>

namespace
{

using namespace rmw::iox2::testing;

class RmwWaitSetTest : public TestBase
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

TEST_F(RmwWaitSetTest, create_and_destroy) {
    auto waitset = rmw_create_wait_set(&context, 0);
    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
}

// This test has risk of being flaky. Virtual clock required.
TEST_F(RmwWaitSetTest, wait_with_timeout) {
    auto waitset = rmw_create_wait_set(&context, 0);

    auto timeout = rmw_time_t{0, 100000000}; // 100ms
    auto start = std::chrono::steady_clock::now();
    ASSERT_RMW_OK(rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, waitset, &timeout));
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    EXPECT_GE(elapsed.count(), 90);  // Allow for some timing inaccuracy
    EXPECT_LT(elapsed.count(), 110); // Allow for some timing inaccuracy

    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
}

TEST_F(RmwWaitSetTest, wakes_up_on_guard_condition_trigger) {
    // create a guard condition
    auto guard_condition = rmw_create_guard_condition(&context);
    auto guard_conditions_array = static_cast<rmw_guard_condition_t**>(rmw_allocate(sizeof(rmw_guard_condition_t)));
    guard_conditions_array[0] = guard_condition;

    // create the guard condition collection struct
    auto guard_conditions = static_cast<rmw_guard_conditions_t*>(rmw_allocate(sizeof(rmw_guard_conditions_t)));
    *guard_conditions = rmw_guard_conditions_t{};
    guard_conditions->guard_conditions = reinterpret_cast<void**>(guard_conditions_array);
    guard_conditions->guard_condition_count = 1;

    // wait on the guard condition
    auto waitset = rmw_create_wait_set(&context, 0);

    // trigger guard condition
    std::thread([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ASSERT_RMW_OK(rmw_trigger_guard_condition(guard_condition));
    }).detach();

    auto timeout = rmw_time_t{0, 0}; // no timeout, stalling test indicates a bug, could be done better..
    ASSERT_RMW_OK(rmw_wait(nullptr, guard_conditions, nullptr, nullptr, nullptr, waitset, &timeout));

    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
    ASSERT_RMW_OK(rmw_destroy_guard_condition(guard_condition));
}

TEST_F(RmwWaitSetTest, wakes_up_on_message_sent_to_subscriber) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    // create publisher
    auto publisher = rmw_create_publisher(test_node(), test_type_support<Defaults>(), test_topic(), nullptr, nullptr);
}

} // namespace
