// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/error_handling.h"
#include "rmw/ret_types.h"
#include "rmw_iceoryx2_cxx/impl/common/error_message.hpp"

#define ASSERT_RMW_OK(expr)                                                                                            \
    do {                                                                                                               \
        rmw_ret_t result = expr;                                                                                       \
        if (result != RMW_RET_OK) {                                                                                    \
            auto msg = std::string(rcutils_get_error_state()->message);                                                \
            rcutils_reset_error();                                                                                     \
            FAIL() << msg;                                                                                             \
        }                                                                                                              \
    } while (0)

#define ASSERT_RMW_ERR(err, expr)                                                                                      \
    do {                                                                                                               \
        rmw_ret_t result = expr;                                                                                       \
        if (result != err) {                                                                                           \
            FAIL() << "Expected return code: " << err << "; Got: " << result;                                          \
        }                                                                                                              \
        rcutils_reset_error();                                                                                         \
    } while (0)

#define EXPECT_RMW_OK(expr)                                                                                            \
    do {                                                                                                               \
        rmw_ret_t result = (expr);                                                                                     \
        if (result != RMW_RET_OK) {                                                                                    \
            auto msg = std::string(rcutils_get_error_state()->message);                                                \
            rcutils_reset_error();                                                                                     \
            ADD_FAILURE() << msg;                                                                                      \
        }                                                                                                              \
    } while (0)

#define EXPECT_RMW_ERR(err, expr)                                                                                      \
    do {                                                                                                               \
        rmw_ret_t result = (expr);                                                                                     \
        if (result != err) {                                                                                           \
            ADD_FAILURE() << "Expected return code: " << err << "; Got: " << result;                                   \
        }                                                                                                              \
        rcutils_reset_error();                                                                                         \
    } while (0)

#define EXPECT_NULLPTR_WITH_RMW_ERR(expr)                                                                              \
    [&]() -> decltype(expr) {                                                                                          \
        auto result = (expr);                                                                                          \
        if (result != nullptr) {                                                                                       \
            ADD_FAILURE() << "Expected nullptr but was: " << result;                                                   \
        }                                                                                                              \
        auto err = rcutils_get_error_state();                                                                          \
        if (!err) {                                                                                                    \
            ADD_FAILURE() << "Expected error but there was none";                                                      \
        }                                                                                                              \
        rcutils_reset_error();                                                                                         \
        return result;                                                                                                 \
    }()


#define RMW_ASSERT_EQ(val1, val2) RMW_ASSERT_PRED_FORMAT2(::testing::internal::EqHelper::Compare, val1, val2)
#define RMW_ASSERT_NE(val1, val2) RMW_ASSERT_PRED_FORMAT2(::testing::internal::CmpHelperNE, val1, val2)

#define RMW_ASSERT_PRED_FORMAT2(pred_format, v1, v2)                                                                   \
    ASSERT_PRED_FORMAT2(pred_format, v1, v2) << (rcutils_error_is_set() ? rcutils_get_error_string().str : "")

#define RMW_ASSERT_TRUE(condition)                                                                                     \
    ASSERT_TRUE(condition) << (rcutils_error_is_set() ? rcutils_get_error_string().str : "")

#define RMW_ASSERT_FALSE(condition)                                                                                    \
    ASSERT_FALSE(condition) << (rcutils_error_is_set() ? rcutils_get_error_string().str : "")
