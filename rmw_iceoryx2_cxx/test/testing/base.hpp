// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/log.hpp"
#include "rcutils/allocator.h"
#include "rmw/error_handling.h"
#include "rmw_iceoryx2_cxx/error_handling.hpp"

#include <random>

namespace rmw::iox2::testing
{

class TestBase : public ::testing::Test
{
protected:
    TestBase() {
        allocator = rcutils_get_default_allocator();
    }

    static void SetUpTestSuite() {
        ::iox2::set_log_level(::iox2::LogLevel::DEBUG);
    }

    void SetUp() override {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<uint32_t> dis(0, std::numeric_limits<uint32_t>::max());
        unique_id = dis(gen);
    }

    void TearDown() override {
        auto error_msg = RMW_IOX2_GET_ERROR_MSG();
        if (error_msg != "") {
            std::cerr << error_msg;
        }
    }

    std::string test_node_name() {
        return std::string("::test_node::" + std::to_string(unique_id));
    }

protected:
    rcutils_allocator_t allocator;
    uint32_t unique_id{0}; // avoid collisions between test cases
};

} // namespace rmw::iox2::testing
