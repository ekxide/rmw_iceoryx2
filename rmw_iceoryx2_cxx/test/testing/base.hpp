// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>

#include "assertions.hpp"
#include "iox2/log.hpp"
#include "rcutils/allocator.h"
#include "rmw/rmw.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

namespace rmw::iox2::testing
{

class TestBase : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {
        ::iox2::set_log_level(::iox2::LogLevel::Debug);
    }

    TestBase() {
        m_allocator = rcutils_get_default_allocator();
        m_unique_id = ::testing::UnitTest::GetInstance()->current_test_case()->total_test_count();

        m_test_topic = "/TestTopic" + std::to_string(m_unique_id);
    }

    rcutils_allocator_t& test_allocator() {
        return m_allocator;
    }

    uint32_t test_id() {
        return m_unique_id;
    }

    rmw_context_t* test_context() {
        return &m_test_context;
    }

    rmw_node_t* test_node() {
        return m_test_node;
    }

    template <typename MessageT>
    const rosidl_message_type_support_t* test_type_support() {
        return rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
    }

    void initialize() {
        initialize_test_context();
        initialize_test_node();
    }

    void initialize_test_context() {
        m_init_options = rmw_get_zero_initialized_init_options();
        ASSERT_RMW_OK(rmw_init_options_init(&m_init_options, m_allocator));
        m_init_options.instance_id = test_id();
        m_test_context = rmw_get_zero_initialized_context();
        ASSERT_RMW_OK(rmw_init(&m_init_options, &m_test_context));
    }

    void initialize_test_node() {
        m_test_node = rmw_create_node(&m_test_context, ("TestNode" + std::to_string(m_unique_id)).c_str(), "/RmwTest");
    }

    std::string create_test_topic(const std::string& name = "") {
        return "/TestId_" + std::to_string(test_id()) + name;
    }

    rmw_node_t* create_default_node(const std::string& node_name) {
        auto node = rmw_create_node(&m_test_context, node_name.c_str(), "/RmwTest");
        m_nodes.push_back(node);
        return node;
    }

    template <typename MessageType>
    rmw_publisher_t* create_default_publisher(const std::string& topic_name) {
        auto pub = rmw_create_publisher(test_node(),
                                        test_type_support<MessageType>(),
                                        topic_name.c_str(),
                                        &rmw_qos_profile_default,
                                        &m_publisher_options);
        m_publishers.push_back(pub);
        return pub;
    }

    template <typename MessageType>
    rmw_subscription_t* create_default_subscriber(const std::string& topic_name) {
        auto sub = rmw_create_subscription(test_node(),
                                           test_type_support<MessageType>(),
                                           topic_name.c_str(),
                                           &rmw_qos_profile_default,
                                           &m_subscriber_options);
        m_subscribers.push_back(sub);
        return sub;
    }

    void cleanup_endpoints() {
        for (auto pub : m_publishers) {
            EXPECT_RMW_OK(rmw_destroy_publisher(test_node(), pub));
        }
        m_publishers.clear();
        for (auto sub : m_subscribers) {
            EXPECT_RMW_OK(rmw_destroy_subscription(test_node(), sub));
        }
        m_subscribers.clear();
    }

    void cleanup_test_context() {
        EXPECT_RMW_OK(rmw_shutdown(&m_test_context));
        EXPECT_RMW_OK(rmw_context_fini(&m_test_context));
        EXPECT_RMW_OK(rmw_init_options_fini(&m_init_options));
    }

    void cleanup_test_node() {
        EXPECT_RMW_OK(rmw_destroy_node(m_test_node));
    }

    void cleanup() {
        cleanup_test_node();
        cleanup_test_context();
    }

    void print_rmw_errors() {
        if (rcutils_error_is_set()) {
            std::cerr << rcutils_get_error_string().str;
        }
    }

private:
    rcutils_allocator_t m_allocator;
    rmw_init_options_t m_init_options;
    rmw_context_t m_test_context;
    rmw_node_t* m_test_node;
    std::string m_test_topic;
    std::vector<rmw_node_t*> m_nodes;
    const rmw_publisher_options_t m_publisher_options{rmw_get_default_publisher_options()};
    std::vector<rmw_publisher_t*> m_publishers;
    const rmw_subscription_options_t m_subscriber_options{rmw_get_default_subscription_options()};
    std::vector<rmw_subscription_t*> m_subscribers;

private:
    uint32_t m_unique_id{0}; // avoid collisions between test cases
};

namespace names
{
std::string test_handle(uint32_t test_id);
}

} // namespace rmw::iox2::testing
