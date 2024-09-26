// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/rmw.h"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

class RmwGraphTest : public rmw::iox2::testing::TestBase
{
protected:
    void SetUp() override {
        allocator = rcutils_get_default_allocator();
        init_options = rmw_get_zero_initialized_init_options();
        ASSERT_RMW_OK(rmw_init_options_init(&init_options, allocator));
        context = rmw_get_zero_initialized_context();
        ASSERT_RMW_OK(rmw_init(&init_options, &context));
    }

    void TearDown() override {
        ASSERT_RMW_OK(rmw_shutdown(&context));
        ASSERT_RMW_OK(rmw_context_fini(&context));
        ASSERT_RMW_OK(rmw_init_options_fini(&init_options));
    }

protected:
    bool rcutils_string_array_contains(const rcutils_string_array_t* array, const char* str) {
        if (!array || !str)
            return false;
        for (size_t i = 0; i < array->size; ++i) {
            if (strcmp(array->data[i], str) == 0) {
                return true;
            }
        }
        return false;
    }

    bool contains_node_name_and_namespace(const char* node_name,
                                          const char* node_namespace,
                                          const rcutils_string_array_t& names,
                                          const rcutils_string_array_t& namespaces) {
        for (size_t i = 0; i < names.size; ++i) {
            if (strcmp(names.data[i], node_name) == 0 && strcmp(namespaces.data[i], node_namespace) == 0) {
                return true;
            }
        }
        return false;
    }

protected:
    rcutils_allocator_t allocator;
    rmw_init_options_t init_options;
    rmw_context_t context;
};

TEST_F(RmwGraphTest, CanListNodes) {
    auto camera_node = rmw_create_node(&context, "Camera", "Sensors");
    auto lidar_node = rmw_create_node(&context, "Lidar", "Sensors");
    auto perception_node = rmw_create_node(&context, "Perception", "ADAS");

    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();

    EXPECT_RMW_OK(rmw_get_node_names(nullptr, &names, &namespaces));

    ASSERT_GE(names.size, 3u);
    ASSERT_GE(namespaces.size, 3u);
    ASSERT_TRUE(contains_node_name_and_namespace("Camera", "Sensors", names, namespaces));
    ASSERT_TRUE(contains_node_name_and_namespace("Lidar", "Sensors", names, namespaces));
    ASSERT_TRUE(contains_node_name_and_namespace("Perception", "ADAS", names, namespaces));

    ASSERT_RMW_OK(rcutils_string_array_fini(&names));
    ASSERT_RMW_OK(rcutils_string_array_fini(&namespaces));

    ASSERT_RMW_OK(rmw_destroy_node(camera_node));
    ASSERT_RMW_OK(rmw_destroy_node(lidar_node));
    ASSERT_RMW_OK(rmw_destroy_node(perception_node));
}

} // namespace
