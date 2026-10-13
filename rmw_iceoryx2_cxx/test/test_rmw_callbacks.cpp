// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/basic_types.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <condition_variable>
#include <cstddef>
#include <mutex>

namespace
{

using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
using rmw_iceoryx2_cxx_test_msgs::srv::BasicTypes;

struct Calls
{
    std::mutex mutex;
    std::condition_variable called;
    size_t events{0};
    size_t calls{0};
};

void count_events(const void* user_data, size_t number_of_events) {
    auto* calls = static_cast<Calls*>(const_cast<void*>(user_data));
    std::lock_guard<std::mutex> lock{calls->mutex};
    calls->events += number_of_events;
    ++calls->calls;
    calls->called.notify_all();
}

class RmwCallbackTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }

    static auto events_of(Calls& calls) -> size_t {
        std::lock_guard<std::mutex> lock{calls.mutex};
        return calls.events;
    }

    static void wait_for_events(Calls& calls, size_t events) {
        std::unique_lock<std::mutex> lock{calls.mutex};
        calls.called.wait(lock, [&] { return calls.events >= events; });
    }

    void send_request(rmw_client_t* client) {
        BasicTypes::Request request;
        int64_t sequence_id = 0;
        ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
    }
};

TEST_F(RmwCallbackTest, new_message_callback_reports_the_messages_published_before_it_was_set) {
    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscription, nullptr);
    for (int i = 0; i < 3; ++i) {
        auto message = Defaults{};
        ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    }

    Calls calls;
    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, count_events, &calls));
    EXPECT_EQ(events_of(calls), 3U);

    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, nullptr, nullptr));
}

TEST_F(RmwCallbackTest, new_message_callback_is_called_for_every_message) {
    constexpr size_t MESSAGES = 100;
    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscription, nullptr);

    Calls calls;
    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, count_events, &calls));
    for (size_t i = 1; i <= MESSAGES; ++i) {
        auto message = Defaults{};
        ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
        wait_for_events(calls, i);
    }
    EXPECT_EQ(events_of(calls), MESSAGES);

    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, nullptr, nullptr));
}

TEST_F(RmwCallbackTest, messages_published_without_a_callback_go_to_the_next_callback) {
    auto* publisher = create_default_publisher<Defaults>(create_test_topic());
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    ASSERT_NE(publisher, nullptr);
    ASSERT_NE(subscription, nullptr);

    Calls first;
    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, count_events, &first));
    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, nullptr, nullptr));
    for (int i = 0; i < 2; ++i) {
        auto message = Defaults{};
        ASSERT_RMW_OK(rmw_publish(publisher, &message, nullptr));
    }

    Calls second;
    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, count_events, &second));
    EXPECT_EQ(events_of(first), 0U);
    EXPECT_EQ(events_of(second), 2U);

    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, nullptr, nullptr));
}

TEST_F(RmwCallbackTest, new_request_callback_reports_the_requests_sent_before_it_was_set) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    ASSERT_NE(service, nullptr);
    ASSERT_NE(client, nullptr);
    send_request(client);
    send_request(client);

    Calls calls;
    ASSERT_RMW_OK(rmw_service_set_on_new_request_callback(service, count_events, &calls));
    EXPECT_EQ(events_of(calls), 2U);

    ASSERT_RMW_OK(rmw_service_set_on_new_request_callback(service, nullptr, nullptr));
}

TEST_F(RmwCallbackTest, new_request_callback_is_called_for_every_request) {
    constexpr size_t REQUESTS = ::rmw::iox2::DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT;
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    ASSERT_NE(service, nullptr);
    ASSERT_NE(client, nullptr);

    Calls calls;
    ASSERT_RMW_OK(rmw_service_set_on_new_request_callback(service, count_events, &calls));
    for (size_t i = 1; i <= REQUESTS; ++i) {
        send_request(client);
        wait_for_events(calls, i);
        BasicTypes::Request request;
        rmw_service_info_t header{};
        bool taken = false;
        ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
        ASSERT_TRUE(taken);
    }
    EXPECT_EQ(events_of(calls), REQUESTS);

    ASSERT_RMW_OK(rmw_service_set_on_new_request_callback(service, nullptr, nullptr));
}

TEST_F(RmwCallbackTest, new_response_callback_is_called_when_a_response_arrives) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    ASSERT_NE(service, nullptr);
    ASSERT_NE(client, nullptr);

    send_request(client);
    BasicTypes::Request request;
    rmw_service_info_t header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    ASSERT_TRUE(taken);
    Calls calls;
    ASSERT_RMW_OK(rmw_client_set_on_new_response_callback(client, count_events, &calls));
    const auto events_before_the_response = events_of(calls);
    BasicTypes::Response response;
    ASSERT_RMW_OK(rmw_send_response(service, &header.request_id, &response));

    wait_for_events(calls, events_before_the_response + 1);
    ASSERT_RMW_OK(rmw_take_response(client, &header, &response, &taken));
    EXPECT_TRUE(taken);

    ASSERT_RMW_OK(rmw_client_set_on_new_response_callback(client, nullptr, nullptr));
}

TEST_F(RmwCallbackTest, callbacks_can_be_cleared_after_the_context_was_shut_down) {
    auto* subscription = create_default_subscriber<Defaults>(create_test_topic());
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    ASSERT_NE(subscription, nullptr);
    ASSERT_NE(service, nullptr);
    ASSERT_NE(client, nullptr);
    Calls calls;
    ASSERT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, count_events, &calls));
    ASSERT_RMW_OK(rmw_service_set_on_new_request_callback(service, count_events, &calls));
    ASSERT_RMW_OK(rmw_client_set_on_new_response_callback(client, count_events, &calls));

    ASSERT_RMW_OK(rmw_shutdown(test_context()));

    EXPECT_RMW_OK(rmw_subscription_set_on_new_message_callback(subscription, nullptr, nullptr));
    EXPECT_RMW_OK(rmw_service_set_on_new_request_callback(service, nullptr, nullptr));
    EXPECT_RMW_OK(rmw_client_set_on_new_response_callback(client, nullptr, nullptr));
}

} // namespace
