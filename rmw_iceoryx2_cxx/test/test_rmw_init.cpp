// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/rmw/identifier.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

namespace
{

using namespace rmw::iox2::testing;

class RmwInitTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(RmwInitTest, initialization_and_shutdown) {
    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    rmw_context_t context = rmw_get_zero_initialized_context();

    EXPECT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    EXPECT_RMW_OK(rmw_init(&init_options, &context));

    EXPECT_EQ(context.implementation_identifier, rmw_get_implementation_identifier());
    EXPECT_NE(context.impl, nullptr);

    EXPECT_RMW_OK(rmw_shutdown(&context));
    EXPECT_RMW_OK(rmw_context_fini(&context));
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

class RmwInitQosOptionsTest : public TestBase
{
protected:
    void SetUp() override {
        unset_environment("RMW_IOX2_QOS_MATCHING");
        unset_environment("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_NODES_PER_SERVICE");
    }

    void TearDown() override {
        restore_environment();
        print_rmw_errors();
    }

private:
    void unset_environment(const char* name) {
        const char* current = std::getenv(name);
        m_snapshots.emplace_back(
            name, current == nullptr ? std::optional<std::string>{} : std::optional<std::string>{current});
        unsetenv(name);
    }

    void restore_environment() {
        for (const auto& [name, value] : m_snapshots) {
            if (value.has_value()) {
                setenv(name.c_str(), value->c_str(), 1);
            } else {
                unsetenv(name.c_str());
            }
        }
        m_snapshots.clear();
    }

    std::vector<std::pair<std::string, std::optional<std::string>>> m_snapshots;
};

// Happy path ----------------------------------------------------------------

TEST_F(RmwInitQosOptionsTest, uses_defaults_when_env_unset) {
    setenv("RMW_IOX2_QOS_MATCHING", "", 1);
    setenv("RMW_IOX2_MAX_NODES_PER_SERVICE", "", 1);
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "", 1);
    setenv("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC", "", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_NE(init_options.impl, nullptr);

    EXPECT_EQ(init_options.impl->qos_matching_mode, ::rmw::iox2::QosMatchingMode::STRICT);
    EXPECT_FALSE(init_options.impl->max_publishers_per_topic.has_value());
    EXPECT_FALSE(init_options.impl->max_subscribers_per_topic.has_value());
    EXPECT_FALSE(init_options.impl->max_nodes_per_service.has_value());

    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitQosOptionsTest, properly_sets_strict_matching_mode) {
    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    EXPECT_EQ(init_options.impl->qos_matching_mode, ::rmw::iox2::QosMatchingMode::STRICT);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitQosOptionsTest, properly_sets_adoptive_matching_mode) {
    setenv("RMW_IOX2_QOS_MATCHING", "adoptive", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    EXPECT_EQ(init_options.impl->qos_matching_mode, ::rmw::iox2::QosMatchingMode::ADOPTIVE);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitQosOptionsTest, properly_sets_max_publishers_per_topic) {
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "64", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_publishers_per_topic.has_value());
    EXPECT_EQ(init_options.impl->max_publishers_per_topic.value(), 64U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitQosOptionsTest, properly_sets_max_subscribers_per_topic) {
    setenv("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC", "128", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_subscribers_per_topic.has_value());
    EXPECT_EQ(init_options.impl->max_subscribers_per_topic.value(), 128U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitQosOptionsTest, properly_sets_max_nodes_per_service) {
    setenv("RMW_IOX2_MAX_NODES_PER_SERVICE", "16", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_nodes_per_service.has_value());
    EXPECT_EQ(init_options.impl->max_nodes_per_service.value(), 16U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

// Error path ----------------------------------------------------------------

TEST_F(RmwInitQosOptionsTest, rejects_invalid_matching_mode) {
    setenv("RMW_IOX2_QOS_MATCHING", "lenient", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_init(&init_options, test_allocator()), RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(init_options.impl, nullptr);
}

TEST_F(RmwInitQosOptionsTest, rejects_invalid_size) {
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "banana", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_init(&init_options, test_allocator()), RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(init_options.impl, nullptr);
}

} // namespace
