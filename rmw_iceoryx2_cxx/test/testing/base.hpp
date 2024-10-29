// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "assertions.hpp"
#include "iox2/log.hpp"
#include "rcutils/allocator.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/error_handling.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include <random>

namespace rmw::iox2::testing
{

class TestBase : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {
        ::iox2::set_log_level(::iox2::LogLevel::Debug);
    }

    TestBase() {
        allocator = rcutils_get_default_allocator();

        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<uint32_t> dis(0, std::numeric_limits<uint32_t>::max());
        m_unique_id = dis(gen);

        m_test_topic = std::to_string(m_unique_id);
    }

    uint32_t test_id() {
        return m_unique_id;
    }

    rmw_node_t* test_node() {
        return m_test_node;
    }

    const char* test_topic() {
        return m_test_topic.c_str();
    }

    template <typename MessageT>
    const rosidl_message_type_support_t* test_type_support() {
        return rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>();
    }

    void initialize_test_context() {
        init_options = rmw_get_zero_initialized_init_options();
        ASSERT_RMW_OK(rmw_init_options_init(&init_options, allocator));
        init_options.instance_id = test_id();
        context = rmw_get_zero_initialized_context();
        ASSERT_RMW_OK(rmw_init(&init_options, &context));
    }

    void initialize_test_node() {
        m_test_node = rmw_create_node(&context, std::to_string(m_unique_id).c_str(), "TestNode");
    }

    void cleanup_test_node() {
        EXPECT_RMW_OK(rmw_destroy_node(m_test_node));
    }

    void cleanup_test_context() {
        EXPECT_RMW_OK(rmw_shutdown(&context));
        EXPECT_RMW_OK(rmw_context_fini(&context));
        EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
    }

    void print_rmw_errors() {
        if (rcutils_error_is_set()) {
            std::cerr << rcutils_get_error_string().str;
        }
    }

protected:
    rcutils_allocator_t allocator;
    rmw_init_options_t init_options;
    rmw_context_t context;
    rmw_node_t* m_test_node;
    std::string m_test_topic;

private:
    uint32_t m_unique_id{0}; // avoid collisions between test cases
};

namespace names
{
std::string test_node(uint32_t test_id);
}

} // namespace rmw::iox2::testing
