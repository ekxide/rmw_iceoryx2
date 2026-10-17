// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/event.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

namespace
{

using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

struct Calls
{
    std::mutex mutex;
    std::condition_variable called;
    size_t calls{0};
    size_t events{0};
};

void count_calls(const void* user_data, size_t number_of_events) {
    auto* calls = static_cast<Calls*>(const_cast<void*>(user_data));
    std::lock_guard<std::mutex> lock{calls->mutex};
    ++calls->calls;
    calls->events += number_of_events;
    calls->called.notify_all();
}

class RmwEventTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }

    auto create_subscription() -> rmw_subscription_t* {
        auto qos = rmw_qos_profile_default;
        auto options = rmw_get_default_subscription_options();
        return rmw_create_subscription(
            test_node(), test_type_support<Defaults>(), create_test_topic().c_str(), &qos, &options);
    }

    auto create_publisher() -> rmw_publisher_t* {
        auto qos = rmw_qos_profile_default;
        auto options = rmw_get_default_publisher_options();
        return rmw_create_publisher(
            test_node(), test_type_support<Defaults>(), create_test_topic().c_str(), &qos, &options);
    }

    static auto take(rmw_event_t& event) -> rmw_matched_status_t {
        rmw_matched_status_t status{};
        bool taken{false};
        EXPECT_RMW_OK(rmw_take_event(&event, &status, &taken));
        EXPECT_TRUE(taken);
        return status;
    }

    auto is_ready(rmw_event_t& event) -> bool {
        void* events[1] = {&event};
        rmw_events_t rmw_events{1, events};
        auto* wait_set = rmw_create_wait_set(test_context(), 1);
        rmw_time_t no_wait{0, 0};
        auto result = rmw_wait(nullptr, nullptr, nullptr, nullptr, &rmw_events, wait_set, &no_wait);
        EXPECT_TRUE(result == RMW_RET_OK || result == RMW_RET_TIMEOUT);
        EXPECT_RMW_OK(rmw_destroy_wait_set(wait_set));
        return events[0] != nullptr;
    }

    static void wait_for_calls(Calls& calls, size_t number_of_calls) {
        std::unique_lock<std::mutex> lock{calls.mutex};
        calls.called.wait(lock, [&] { return calls.calls >= number_of_calls; });
    }
};

TEST_F(RmwEventTest, only_matched_events_are_supported) {
    EXPECT_TRUE(rmw_event_type_is_supported(RMW_EVENT_PUBLICATION_MATCHED));
    EXPECT_TRUE(rmw_event_type_is_supported(RMW_EVENT_SUBSCRIPTION_MATCHED));
    for (const auto type : {RMW_EVENT_LIVELINESS_CHANGED,
                            RMW_EVENT_REQUESTED_DEADLINE_MISSED,
                            RMW_EVENT_REQUESTED_QOS_INCOMPATIBLE,
                            RMW_EVENT_MESSAGE_LOST,
                            RMW_EVENT_SUBSCRIPTION_INCOMPATIBLE_TYPE,
                            RMW_EVENT_LIVELINESS_LOST,
                            RMW_EVENT_OFFERED_DEADLINE_MISSED,
                            RMW_EVENT_OFFERED_QOS_INCOMPATIBLE,
                            RMW_EVENT_PUBLISHER_INCOMPATIBLE_TYPE,
                            RMW_EVENT_INVALID}) {
        EXPECT_FALSE(rmw_event_type_is_supported(type)) << "event type " << type;
    }
}

TEST_F(RmwEventTest, unsupported_event_types_are_refused) {
    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);

    auto event = rmw_get_zero_initialized_event();
    EXPECT_EQ(rmw_publisher_event_init(&event, publisher, RMW_EVENT_LIVELINESS_LOST), RMW_RET_UNSUPPORTED);

    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

TEST_F(RmwEventTest, publication_matched_reports_subscriptions_joining_and_leaving) {
    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);
    auto event = rmw_get_zero_initialized_event();
    ASSERT_RMW_OK(rmw_publisher_event_init(&event, publisher, RMW_EVENT_PUBLICATION_MATCHED));

    auto status = take(event);
    EXPECT_EQ(status.total_count, 0U);
    EXPECT_EQ(status.current_count, 0U);

    auto* first = create_subscription();
    auto* second = create_subscription();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    status = take(event);
    EXPECT_EQ(status.total_count, 2U);
    EXPECT_EQ(status.total_count_change, 2U);
    EXPECT_EQ(status.current_count, 2U);
    EXPECT_EQ(status.current_count_change, 2);

    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), first));
    status = take(event);
    EXPECT_EQ(status.total_count, 2U);
    EXPECT_EQ(status.total_count_change, 0U);
    EXPECT_EQ(status.current_count, 1U);
    EXPECT_EQ(status.current_count_change, -1);

    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), second));
    ASSERT_RMW_OK(rmw_event_fini(&event));
    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

TEST_F(RmwEventTest, subscription_matched_reports_publishers_joining_and_leaving) {
    auto* subscription = create_subscription();
    ASSERT_NE(subscription, nullptr);
    auto event = rmw_get_zero_initialized_event();
    ASSERT_RMW_OK(rmw_subscription_event_init(&event, subscription, RMW_EVENT_SUBSCRIPTION_MATCHED));

    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);
    auto status = take(event);
    EXPECT_EQ(status.total_count, 1U);
    EXPECT_EQ(status.current_count, 1U);
    EXPECT_EQ(status.current_count_change, 1);

    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
    status = take(event);
    EXPECT_EQ(status.total_count, 1U);
    EXPECT_EQ(status.current_count, 0U);
    EXPECT_EQ(status.current_count_change, -1);

    ASSERT_RMW_OK(rmw_event_fini(&event));
    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
}

TEST_F(RmwEventTest, a_matched_event_is_ready_until_the_change_is_taken) {
    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);
    auto event = rmw_get_zero_initialized_event();
    ASSERT_RMW_OK(rmw_publisher_event_init(&event, publisher, RMW_EVENT_PUBLICATION_MATCHED));
    (void)take(event);
    EXPECT_FALSE(is_ready(event));

    auto* subscription = create_subscription();
    ASSERT_NE(subscription, nullptr);
    EXPECT_TRUE(is_ready(event));
    EXPECT_TRUE(is_ready(event));

    (void)take(event);
    EXPECT_FALSE(is_ready(event));

    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
    ASSERT_RMW_OK(rmw_event_fini(&event));
    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

TEST_F(RmwEventTest, concurrent_takes_of_a_matched_event_count_every_subscription_once) {
    constexpr int NUM_THREADS = 4;
    constexpr int NUM_TAKES = 2000;
    constexpr int NUM_SUBSCRIPTIONS = 16;

    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);
    auto event = rmw_get_zero_initialized_event();
    ASSERT_RMW_OK(rmw_publisher_event_init(&event, publisher, RMW_EVENT_PUBLICATION_MATCHED));

    std::atomic<size_t> joined{0};
    auto take_and_count = [&] {
        rmw_matched_status_t status{};
        bool taken{false};
        EXPECT_EQ(rmw_take_event(&event, &status, &taken), RMW_RET_OK);
        joined += status.total_count_change;
    };
    std::vector<std::thread> threads;
    for (int t = 0; t < NUM_THREADS; ++t) {
        threads.emplace_back([&] {
            for (int i = 0; i < NUM_TAKES; ++i) {
                take_and_count();
            }
        });
    }
    std::vector<rmw_subscription_t*> subscriptions;
    for (int i = 0; i < NUM_SUBSCRIPTIONS; ++i) {
        subscriptions.push_back(create_subscription());
        EXPECT_NE(subscriptions.back(), nullptr);
    }
    for (auto& thread : threads) {
        thread.join();
    }
    take_and_count();
    EXPECT_EQ(joined.load(), static_cast<size_t>(NUM_SUBSCRIPTIONS));

    for (auto* subscription : subscriptions) {
        EXPECT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
    }
    ASSERT_RMW_OK(rmw_event_fini(&event));
    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

TEST_F(RmwEventTest, the_matched_callback_is_called_once_for_every_change) {
    constexpr size_t CHANGES_AFTER_MATCH = 1;
    constexpr size_t CHANGES_AFTER_UNMATCH = 2;

    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);
    auto event = rmw_get_zero_initialized_event();
    ASSERT_RMW_OK(rmw_publisher_event_init(&event, publisher, RMW_EVENT_PUBLICATION_MATCHED));
    Calls calls;
    ASSERT_RMW_OK(rmw_event_set_callback(&event, count_calls, &calls));

    auto* subscription = create_subscription();
    ASSERT_NE(subscription, nullptr);
    wait_for_calls(calls, CHANGES_AFTER_MATCH);
    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), subscription));
    wait_for_calls(calls, CHANGES_AFTER_UNMATCH);
    {
        std::lock_guard<std::mutex> lock{calls.mutex};
        EXPECT_EQ(calls.calls, CHANGES_AFTER_UNMATCH);
        EXPECT_EQ(calls.events, CHANGES_AFTER_UNMATCH);
    }

    ASSERT_RMW_OK(rmw_event_set_callback(&event, nullptr, nullptr));
    ASSERT_RMW_OK(rmw_event_fini(&event));
    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

TEST_F(RmwEventTest, the_matched_callback_reports_the_changes_since_the_last_take) {
    constexpr size_t CALLS_FOR_EARLIER_CHANGES = 1;
    constexpr size_t MATCHED_SUBSCRIPTIONS = 2;

    auto* publisher = create_publisher();
    ASSERT_NE(publisher, nullptr);
    auto event = rmw_get_zero_initialized_event();
    ASSERT_RMW_OK(rmw_publisher_event_init(&event, publisher, RMW_EVENT_PUBLICATION_MATCHED));
    auto* first = create_subscription();
    auto* second = create_subscription();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);

    Calls calls;
    ASSERT_RMW_OK(rmw_event_set_callback(&event, count_calls, &calls));
    {
        std::lock_guard<std::mutex> lock{calls.mutex};
        EXPECT_EQ(calls.calls, CALLS_FOR_EARLIER_CHANGES);
        EXPECT_EQ(calls.events, MATCHED_SUBSCRIPTIONS);
    }

    ASSERT_RMW_OK(rmw_event_set_callback(&event, nullptr, nullptr));
    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), second));
    ASSERT_RMW_OK(rmw_destroy_subscription(test_node(), first));
    ASSERT_RMW_OK(rmw_event_fini(&event));
    ASSERT_RMW_OK(rmw_destroy_publisher(test_node(), publisher));
}

} // namespace
