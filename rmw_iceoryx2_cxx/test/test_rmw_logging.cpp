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
#include "rmw/rmw.h"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwLoggingTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }

protected:
};

TEST_F(RmwLoggingTest, set_log_severity) {
    ASSERT_RMW_OK(rmw_set_log_severity(RMW_LOG_SEVERITY_DEBUG));
    ASSERT_EQ(iox2::get_log_level(), iox2::LogLevel::Debug);

    ASSERT_RMW_OK(rmw_set_log_severity(RMW_LOG_SEVERITY_INFO));
    ASSERT_EQ(iox2::get_log_level(), iox2::LogLevel::Info);

    ASSERT_RMW_OK(rmw_set_log_severity(RMW_LOG_SEVERITY_WARN));
    ASSERT_EQ(iox2::get_log_level(), iox2::LogLevel::Warn);

    ASSERT_RMW_OK(rmw_set_log_severity(RMW_LOG_SEVERITY_ERROR));
    ASSERT_EQ(iox2::get_log_level(), iox2::LogLevel::Error);

    ASSERT_RMW_OK(rmw_set_log_severity(RMW_LOG_SEVERITY_FATAL));
    ASSERT_EQ(iox2::get_log_level(), iox2::LogLevel::Fatal);
}

} // namespace
