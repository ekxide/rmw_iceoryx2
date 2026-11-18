// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rcutils/strdup.h"
#include "rmw/discovery_options.h"
#include "rmw/init.h"
#include "rmw/init_options.h"
#include "rmw/security_options.h"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/rmw/identifier.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"
#include "testing/failing_allocator.hpp"

#include <cstdlib>
#include <limits>
#include <string>

namespace
{

using namespace rmw::iox2::testing;

constexpr size_t UNLIMITED_ALLOCATIONS = std::numeric_limits<size_t>::max();
constexpr size_t NO_ALLOCATIONS = 0;
constexpr size_t ALLOCATIONS_BEFORE_ENCLAVE = 0;
constexpr size_t ALLOCATIONS_BEFORE_SECURITY_ROOT_PATH = 1;
constexpr size_t ALLOCATIONS_BEFORE_STATIC_PEERS = 2;
constexpr size_t NO_STATIC_PEERS = 0;
constexpr size_t STATIC_PEER_COUNT = 1;
constexpr const char* SECURITY_ROOT_PATH = "/keystore";
constexpr const char* NO_SECURITY_ROOT_PATH = nullptr;

auto deallocate_non_null(void* pointer, void* state) -> void {
    EXPECT_NE(pointer, nullptr);
    failing_deallocate(pointer, state);
}

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

class RmwInitOptionsTest : public TestBase
{
protected:
    void SetUp() override {
        unset_environment("RMW_IOX2_QOS_MATCHING");
        unset_environment("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC");
        unset_environment("RMW_IOX2_MAX_NODES_PER_SERVICE");
        unset_environment("RMW_IOX2_MAX_DEPTH_PER_TOPIC");
    }

    void TearDown() override {
        restore_environment();
        print_rmw_errors();
    }
};

// Happy path ----------------------------------------------------------------

TEST_F(RmwInitOptionsTest, uses_defaults_when_env_unset) {
    setenv("RMW_IOX2_QOS_MATCHING", "", 1);
    setenv("RMW_IOX2_MAX_NODES_PER_SERVICE", "", 1);
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "", 1);
    setenv("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC", "", 1);
    setenv("RMW_IOX2_MAX_DEPTH_PER_TOPIC", "", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_NE(init_options.impl, nullptr);

    EXPECT_EQ(init_options.impl->qos_matching_mode, ::rmw::iox2::QosMatchingMode::ROS);
    EXPECT_FALSE(init_options.impl->max_publishers_per_topic.has_value());
    EXPECT_FALSE(init_options.impl->max_subscribers_per_topic.has_value());
    EXPECT_FALSE(init_options.impl->max_nodes_per_service.has_value());
    EXPECT_FALSE(init_options.impl->max_depth_per_topic.has_value());
    EXPECT_EQ(init_options.domain_id, RMW_DEFAULT_DOMAIN_ID);

    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitOptionsTest, properly_sets_strict_matching_mode) {
    setenv("RMW_IOX2_QOS_MATCHING", "strict", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    EXPECT_EQ(init_options.impl->qos_matching_mode, ::rmw::iox2::QosMatchingMode::STRICT);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitOptionsTest, properly_sets_adoptive_matching_mode) {
    setenv("RMW_IOX2_QOS_MATCHING", "adoptive", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    EXPECT_EQ(init_options.impl->qos_matching_mode, ::rmw::iox2::QosMatchingMode::ADOPTIVE);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitOptionsTest, properly_sets_max_publishers_per_topic) {
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "64", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_publishers_per_topic.has_value());
    EXPECT_EQ(init_options.impl->max_publishers_per_topic.value(), 64U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitOptionsTest, properly_sets_max_subscribers_per_topic) {
    setenv("RMW_IOX2_MAX_SUBSCRIBERS_PER_TOPIC", "128", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_subscribers_per_topic.has_value());
    EXPECT_EQ(init_options.impl->max_subscribers_per_topic.value(), 128U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitOptionsTest, properly_sets_max_nodes_per_service) {
    setenv("RMW_IOX2_MAX_NODES_PER_SERVICE", "16", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_nodes_per_service.has_value());
    EXPECT_EQ(init_options.impl->max_nodes_per_service.value(), 16U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

TEST_F(RmwInitOptionsTest, properly_sets_max_depth_per_topic) {
    setenv("RMW_IOX2_MAX_DEPTH_PER_TOPIC", "1000", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&init_options, test_allocator()));
    ASSERT_TRUE(init_options.impl->max_depth_per_topic.has_value());
    EXPECT_EQ(init_options.impl->max_depth_per_topic.value(), 1000U);
    EXPECT_RMW_OK(rmw_init_options_fini(&init_options));
}

// Error path ----------------------------------------------------------------

TEST_F(RmwInitOptionsTest, rejects_invalid_matching_mode) {
    setenv("RMW_IOX2_QOS_MATCHING", "lenient", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_init(&init_options, test_allocator()), RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(init_options.impl, nullptr);
}

TEST_F(RmwInitOptionsTest, rejects_invalid_size) {
    setenv("RMW_IOX2_MAX_PUBLISHERS_PER_TOPIC", "banana", 1);

    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    EXPECT_EQ(rmw_init_options_init(&init_options, test_allocator()), RMW_RET_INVALID_ARGUMENT);
    EXPECT_EQ(init_options.impl, nullptr);
}

TEST_F(RmwInitOptionsTest, copies_options_without_enclave) {
    rmw_init_options_t src = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&src, test_allocator()));
    src.allocator.deallocate(src.enclave, src.allocator.state);
    src.enclave = nullptr;

    rmw_init_options_t dst = rmw_get_zero_initialized_init_options();
    EXPECT_RMW_OK(rmw_init_options_copy(&src, &dst));
    EXPECT_EQ(dst.enclave, nullptr);

    EXPECT_RMW_OK(rmw_init_options_fini(&dst));
    EXPECT_RMW_OK(rmw_init_options_fini(&src));
}

TEST_F(RmwInitOptionsTest, copy_shares_no_memory_with_the_source) {
    rmw_init_options_t src = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&src, test_allocator()));
    ASSERT_RMW_OK(rmw_security_options_set_root_path(SECURITY_ROOT_PATH, &src.allocator, &src.security_options));
    src.security_options.enforce_security = RMW_SECURITY_ENFORCEMENT_ENFORCE;
    ASSERT_RMW_OK(rmw_discovery_options_init(&src.discovery_options, STATIC_PEER_COUNT, &src.allocator));

    rmw_init_options_t dst = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_copy(&src, &dst));
    EXPECT_NE(dst.enclave, src.enclave);
    EXPECT_STREQ(dst.enclave, src.enclave);
    EXPECT_NE(dst.security_options.security_root_path, src.security_options.security_root_path);
    EXPECT_STREQ(dst.security_options.security_root_path, SECURITY_ROOT_PATH);
    EXPECT_EQ(dst.security_options.enforce_security, RMW_SECURITY_ENFORCEMENT_ENFORCE);
    EXPECT_NE(dst.discovery_options.static_peers, src.discovery_options.static_peers);
    EXPECT_EQ(dst.discovery_options.static_peers_count, STATIC_PEER_COUNT);

    EXPECT_RMW_OK(rmw_init_options_fini(&dst));
    EXPECT_RMW_OK(rmw_init_options_fini(&src));
}

struct CopyFailure
{
    const char* failing_allocation;
    size_t successful_allocations;
    const char* security_root_path;
    size_t static_peer_count;
};

class RmwInitOptionsCopyFailureTest : public RmwInitOptionsTest, public ::testing::WithParamInterface<CopyFailure>
{
};

TEST_P(RmwInitOptionsCopyFailureTest, frees_what_it_copied_and_leaves_the_destination_zero_initialized) {
    FailingAllocatorState state{UNLIMITED_ALLOCATIONS, NO_ALLOCATIONS};
    rmw_init_options_t src = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&src, failing_allocator(&state)));
    src.security_options.security_root_path = rcutils_strdup(GetParam().security_root_path, src.allocator);
    ASSERT_RMW_OK(rmw_discovery_options_init(&src.discovery_options, GetParam().static_peer_count, &src.allocator));
    const auto live_before_copy = state.live;
    state.remaining = GetParam().successful_allocations;

    rmw_init_options_t dst = rmw_get_zero_initialized_init_options();
    EXPECT_RMW_ERR(RMW_RET_BAD_ALLOC, rmw_init_options_copy(&src, &dst));
    EXPECT_EQ(dst.implementation_identifier, nullptr);
    EXPECT_EQ(dst.impl, nullptr);
    EXPECT_EQ(state.live, live_before_copy);

    EXPECT_RMW_OK(rmw_init_options_fini(&src));
}

INSTANTIATE_TEST_SUITE_P(
    EveryAllocation,
    RmwInitOptionsCopyFailureTest,
    ::testing::Values(
        CopyFailure{"enclave", ALLOCATIONS_BEFORE_ENCLAVE, NO_SECURITY_ROOT_PATH, NO_STATIC_PEERS},
        CopyFailure{"security_root_path", ALLOCATIONS_BEFORE_SECURITY_ROOT_PATH, SECURITY_ROOT_PATH, NO_STATIC_PEERS},
        CopyFailure{"static_peers", ALLOCATIONS_BEFORE_STATIC_PEERS, SECURITY_ROOT_PATH, STATIC_PEER_COUNT}),
    [](const auto& info) { return std::string{info.param.failing_allocation}; });

TEST_F(RmwInitOptionsTest, failed_copy_without_enclave_frees_no_null_pointer) {
    FailingAllocatorState state{UNLIMITED_ALLOCATIONS, NO_ALLOCATIONS};
    rmw_init_options_t src = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&src, failing_allocator(&state)));
    src.allocator.deallocate(src.enclave, src.allocator.state);
    src.enclave = nullptr;
    ASSERT_RMW_OK(rmw_security_options_set_root_path(SECURITY_ROOT_PATH, &src.allocator, &src.security_options));
    src.allocator.deallocate = deallocate_non_null;
    state.remaining = NO_ALLOCATIONS;

    rmw_init_options_t dst = rmw_get_zero_initialized_init_options();
    EXPECT_RMW_ERR(RMW_RET_BAD_ALLOC, rmw_init_options_copy(&src, &dst));

    EXPECT_RMW_OK(rmw_init_options_fini(&src));
}

} // namespace
