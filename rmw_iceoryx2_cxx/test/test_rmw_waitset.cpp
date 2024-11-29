// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/publisher_options.h"
#include "rmw/qos_profiles.h"
#include "rmw/rmw.h"
#include "rmw/subscription_options.h"
#include "rmw_iceoryx2_cxx/iox2/guard_condition_impl.hpp"
#include "rmw_iceoryx2_cxx/iox2/subscriber_impl.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <thread>

namespace
{

using namespace rmw::iox2::testing;
/// @brief Class isolating all the complexities of orechestrating a wait set test.
///
/// By containing the complexity here, the hope is that tests are more readable.
class WaitSetTestContext
{
    using GuardConditionImpl = rmw::iox2::GuardConditionImpl;
    using SubscriberImpl = rmw::iox2::SubscriberImpl;

    // Keeps track of typesupports used by added publishers
    struct PublisherDetails
    {
        const rosidl_message_type_support_t* const typesupport;
        rmw_publisher_t* const publisher;
    };

    // Keeps track of typesupports used by added subscribers
    struct SubscriberDetails
    {
        const rosidl_message_type_support_t* const typesupport;
        rmw_subscription_t* const subscriber;
    };

public:
    WaitSetTestContext(rmw_context_t* rmw_context, rmw_node_t* rmw_node, rmw_time_t timeout)
        : m_rmw_context{rmw_context}
        , m_rmw_node{rmw_node}
        , m_timeout{timeout} {
    }

    ~WaitSetTestContext() {
        for (size_t i = 0; i < m_subscriptions.size(); i++) {
            (void)rmw_destroy_publisher(m_rmw_node, m_publishers.at(i).publisher);
            (void)rmw_destroy_subscription(m_rmw_node, m_subscriptions.at(i).subscriber);
        }
        for (size_t i = 0; i < m_guard_conditions.size(); i++) {
            (void)rmw_destroy_guard_condition(m_guard_conditions.at(i));
        }
        if (m_waitset) {
            (void)rmw_destroy_wait_set(m_waitset);
        }
    }

    bool initialize() {
        m_waitset = rmw_create_wait_set(m_rmw_context, 0);
        return m_waitset != nullptr;
    }

    rmw_wait_set_t* waitset() {
        return m_waitset;
    }

    /// @brief Adds the specified number of guard conditions to the context
    bool add_guard_condition() {
        auto guard_condition = rmw_create_guard_condition(m_rmw_context);
        if (!guard_condition) {
            return false;
        }
        m_guard_conditions.push_back(guard_condition);
        return true;
    }

    /// @brief Constructs the array of guard conditions containing all guard conditions in the context.
    ///        This is expected by rmw_wait().
    rmw_guard_conditions_t* guard_conditions_array() {
        auto guard_condition_data =
            static_cast<GuardConditionImpl**>(rmw_allocate(sizeof(GuardConditionImpl*) * m_guard_conditions.size()));
        for (size_t i = 0; i < m_guard_conditions.size(); i++) {
            guard_condition_data[i] = static_cast<GuardConditionImpl*>(m_guard_conditions.at(i)->data);
        }

        m_guard_condition_array = std::make_unique<rmw_guard_conditions_t>();
        m_guard_condition_array->guard_conditions = reinterpret_cast<void**>(guard_condition_data);
        m_guard_condition_array->guard_condition_count = m_guard_conditions.size();

        return m_guard_condition_array.get();
    }

    /// @brief Get access to the guard condition instances in the context
    inline std::vector<rmw_guard_condition_t*>& guard_conditions() {
        return m_guard_conditions;
    }

    /// @brief Add a publisher-subscriber pair to the context
    bool add_publisher_subscriber(const std::string& topic, const rosidl_message_type_support_t* typesupport) {
        static const rmw_publisher_options_t publisher_options = rmw_get_default_publisher_options();
        static const rmw_subscription_options_t subscription_options = rmw_get_default_subscription_options();
        auto publisher =
            rmw_create_publisher(m_rmw_node, typesupport, topic.c_str(), &rmw_qos_profile_default, &publisher_options);
        if (!publisher) {
            return false;
        }
        m_publishers.push_back(PublisherDetails{typesupport, publisher});
        auto subscriber = rmw_create_subscription(
            m_rmw_node, typesupport, topic.c_str(), &rmw_qos_profile_default, &subscription_options);
        if (!subscriber) {
            return false;
        }
        m_subscriptions.push_back(SubscriberDetails{typesupport, subscriber});
        return true;
    }

    /// @brief Get access to the publisher instances in the context
    inline std::vector<rmw_publisher_t*> publishers() {
        std::vector<rmw_publisher_t*> result;
        std::transform(m_publishers.begin(), m_publishers.end(), std::back_inserter(result), [](const auto& details) {
            return details.publisher;
        });
        return result;
    }

    /// @brief Get access to the subscriber instances in the context
    inline std::vector<rmw_subscription_t*> subscribers() {
        std::vector<rmw_subscription_t*> result;
        std::transform(m_subscriptions.begin(),
                       m_subscriptions.end(),
                       std::back_inserter(result),
                       [](const auto& details) { return details.subscriber; });
        return result;
    }

    /// @brief Constructs the array of subscriptions containing all subscriptions in the context.
    ///        This is expected by rmw_wait().
    rmw_subscriptions_t* subscriptions_array() {
        auto subscriber_data =
            static_cast<SubscriberImpl**>(rmw_allocate(sizeof(SubscriberImpl*) * m_subscriptions.size()));
        for (size_t i = 0; i < m_subscriptions.size(); i++) {
            subscriber_data[i] = static_cast<SubscriberImpl*>(m_subscriptions.at(i).subscriber->data);
        }

        m_subscriptions_array = std::make_unique<rmw_subscriptions_t>();
        m_subscriptions_array->subscribers = reinterpret_cast<void**>(subscriber_data);
        m_subscriptions_array->subscriber_count = m_subscriptions.size();

        return m_subscriptions_array.get();
    }

    /// @brief The timeout specified for the context.
    rmw_time_t* timeout() {
        return &m_timeout;
    }

    /// @brief Triggers all guard conditions after a given delay
    template <typename Duration>
    void trigger_guard_conditions_after(Duration& delay, const std::set<size_t>& indices = {}) {
        std::thread([this, delay, indices]() {
            std::this_thread::sleep_for(delay);
            if (indices.empty()) {
                for (size_t index = 0; index < m_guard_conditions.size(); index++) {
                    ASSERT_RMW_OK(rmw_trigger_guard_condition(m_guard_conditions.at(index)));
                }
            } else {
                for (auto index : indices) {
                    ASSERT_RMW_OK(rmw_trigger_guard_condition(m_guard_conditions.at(index)));
                }
            }
        }).detach();
    }

    /// @brief Triggers subscriptions after a given delay
    template <typename Duration>
    void trigger_subscriptions_after(Duration& delay, const std::set<size_t>& indices = {}) {
        using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
        std::thread([this, delay, indices]() {
            std::this_thread::sleep_for(delay);
            if (indices.empty()) {
                for (size_t i = 0; i < m_publishers.size(); i++) {
                    void* loan = nullptr;
                    EXPECT_RMW_OK(
                        rmw_borrow_loaned_message(m_publishers.at(i).publisher, m_publishers.at(i).typesupport, &loan));
                    new (loan) Defaults{};
                    EXPECT_RMW_OK(rmw_publish_loaned_message(m_publishers.at(i).publisher, loan, nullptr));
                }
            } else {
                for (auto i : indices) {
                    void* loan = nullptr;
                    EXPECT_RMW_OK(
                        rmw_borrow_loaned_message(m_publishers.at(i).publisher, m_publishers.at(i).typesupport, &loan));
                    new (loan) Defaults{};
                    EXPECT_RMW_OK(rmw_publish_loaned_message(m_publishers.at(i).publisher, loan, nullptr));
                }
            }
        }).detach();
    }

private:
    rmw_context_t* m_rmw_context{nullptr};
    rmw_node_t* m_rmw_node{nullptr};
    rmw_wait_set_t* m_waitset{nullptr};

    std::vector<rmw_guard_condition_t*> m_guard_conditions;
    std::unique_ptr<rmw_guard_conditions_t> m_guard_condition_array{nullptr};

    std::vector<PublisherDetails> m_publishers;
    std::vector<SubscriberDetails> m_subscriptions;
    std::unique_ptr<rmw_subscriptions_t> m_subscriptions_array{nullptr};

    rmw_time_t m_timeout;
};

class RmwWaitSetTest : public TestBase
{
protected:
    static constexpr rmw_time_t TIMEOUT_AFTER_20MS{0, 20000000};

protected:
    void SetUp() override {
        initialize();
    }
    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }
};

TEST_F(RmwWaitSetTest, create_and_destroy) {
    auto waitset = rmw_create_wait_set(test_context(), 0);
    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
}

// TODO: This test has risk of being flaky. Virtual clock required.
TEST_F(RmwWaitSetTest, can_wait_with_timeout) {
    // ===== Setup
    auto ctx = WaitSetTestContext{test_context(), test_node(), TIMEOUT_AFTER_20MS};
    if (!ctx.initialize()) {
        FAIL() << "failed to initialize context";
    }

    // ===== Test
    auto start = std::chrono::steady_clock::now();
    auto result = rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, ctx.waitset(), ctx.timeout());
    ASSERT_EQ(result, RMW_RET_TIMEOUT);
    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    EXPECT_GE(elapsed.count(), 19);
    EXPECT_LT(elapsed.count(), 30);
}

TEST_F(RmwWaitSetTest, can_be_triggered_by_guard_condition) {
    using rmw::iox2::GuardConditionImpl;

    // ===== Setup
    constexpr size_t NUM_GUARD_CONDITIONS{3};

    auto ctx = WaitSetTestContext{test_context(), test_node(), TIMEOUT_AFTER_20MS};
    if (!ctx.initialize()) {
        FAIL() << "failed to initialize context";
    }

    for (size_t i = 0; i < NUM_GUARD_CONDITIONS; i++) {
        ASSERT_TRUE(ctx.add_guard_condition()) << "failed to create guard condition";
    }

    // ===== Test
    auto delay = std::chrono::milliseconds(10);
    std::set<size_t> triggered_indices{0, 2};
    ctx.trigger_guard_conditions_after(delay, triggered_indices);

    std::set<size_t> received_indices;
    size_t received_count = 0;
    while (received_count < triggered_indices.size()) {
        auto guard_conditions = ctx.guard_conditions_array();
        if (auto result = rmw_wait(nullptr, guard_conditions, nullptr, nullptr, nullptr, ctx.waitset(), ctx.timeout());
            !(result == RMW_RET_OK || result == RMW_RET_TIMEOUT)) {
            FAIL() << "failed to wait";
        }
        for (size_t i = 0; i < guard_conditions->guard_condition_count; i++) {
            if (guard_conditions->guard_conditions[i] != nullptr) {
                received_indices.emplace(i);
                received_count++;
            }
        }
    }

    ASSERT_EQ(triggered_indices.size(), received_count);
}

TEST_F(RmwWaitSetTest, can_be_triggered_by_message_sent_to_subscriber) {
    using rmw::iox2::SubscriberImpl;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    // ===== Setup
    auto ctx = WaitSetTestContext{test_context(), test_node(), TIMEOUT_AFTER_20MS};
    ctx.initialize();

    ASSERT_TRUE(ctx.add_publisher_subscriber("/TestTopicA", test_type_support<Defaults>()))
        << "failed to create publisher/subscriber pair";
    ASSERT_TRUE(ctx.add_publisher_subscriber("/TestTopicB", test_type_support<Defaults>()))
        << "failed to create publisher/subscriber pair";
    ASSERT_TRUE(ctx.add_publisher_subscriber("/TestTopicC", test_type_support<Defaults>()))
        << "failed to create publisher/subscriber pair";

    // ===== Test
    auto delay = std::chrono::milliseconds(10);
    std::set<size_t> triggered_indices{0, 2};
    ctx.trigger_subscriptions_after(delay, triggered_indices);

    std::set<size_t> received_indices;
    size_t received_count = 0;
    while (received_count < triggered_indices.size()) {
        auto subscriptions = ctx.subscriptions_array();
        if (auto result = rmw_wait(subscriptions, nullptr, nullptr, nullptr, nullptr, ctx.waitset(), ctx.timeout());
            !(result == RMW_RET_OK || result == RMW_RET_TIMEOUT)) {
            FAIL() << "failed to wait";
        }
        for (size_t i = 0; i < subscriptions->subscriber_count; i++) {
            if (subscriptions->subscribers[i] != nullptr) {
                received_indices.emplace(i);
                received_count++;
            }
        }
    }

    ASSERT_EQ(triggered_indices.size(), received_count);
}

TEST_F(RmwWaitSetTest, can_get_triggers_from_all_entity_types_in_single_wait) {
    using rmw::iox2::GuardConditionImpl;
    using rmw::iox2::SubscriberImpl;
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    // ===== Setup
    constexpr size_t NUM_GUARD_CONDITIONS{3};
    constexpr size_t NUM_PUBLISH_SUBSCRIBERS{2};

    auto ctx = WaitSetTestContext{test_context(), test_node(), TIMEOUT_AFTER_20MS};
    if (!ctx.initialize()) {
        FAIL() << "failed to initialize context";
    }

    for (size_t i = 0; i < NUM_GUARD_CONDITIONS; i++) {
        ASSERT_TRUE(ctx.add_guard_condition()) << "failed to create guard condition";
    }
    for (size_t i = 0; i < NUM_PUBLISH_SUBSCRIBERS; i++) {
        ASSERT_TRUE(ctx.add_publisher_subscriber("/TestTopic" + std::to_string(i), test_type_support<Defaults>()))
            << "failed to create publisher/subscriber pair";
    }

    // ===== Test
    auto delay = std::chrono::milliseconds(10);
    ctx.trigger_guard_conditions_after(delay);
    ctx.trigger_subscriptions_after(delay);

    std::set<size_t> received_guard_indices;
    std::set<size_t> received_sub_indices;
    size_t total_received = 0;
    size_t expected_total = NUM_GUARD_CONDITIONS + NUM_PUBLISH_SUBSCRIBERS;

    while (total_received < expected_total) {
        auto subscriptions = ctx.subscriptions_array();
        auto guard_conditions = ctx.guard_conditions_array();

        if (auto result =
                rmw_wait(subscriptions, guard_conditions, nullptr, nullptr, nullptr, ctx.waitset(), ctx.timeout());
            !(result == RMW_RET_OK || result == RMW_RET_TIMEOUT)) {
            FAIL() << "failed to wait";
        }

        for (size_t i = 0; i < guard_conditions->guard_condition_count; i++) {
            if (guard_conditions->guard_conditions[i] != nullptr) {
                received_guard_indices.insert(i);
                total_received++;
            }
        }

        for (size_t i = 0; i < subscriptions->subscriber_count; i++) {
            if (subscriptions->subscribers[i] != nullptr) {
                received_sub_indices.insert(i);
                total_received++;
            }
        }
    }

    ASSERT_EQ(received_guard_indices.size(), NUM_GUARD_CONDITIONS);
    ASSERT_EQ(received_sub_indices.size(), NUM_PUBLISH_SUBSCRIBERS);
}

} // namespace
