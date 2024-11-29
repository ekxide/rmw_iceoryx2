// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/get_topic_names_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

namespace
{

using namespace rmw::iox2::testing;

class RmwGraphTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
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
};

TEST_F(RmwGraphTest, can_get_node_names) {
    auto camera_node = rmw_create_node(test_context(), "Camera", "/Sensors");
    auto lidar_node = rmw_create_node(test_context(), "Lidar", "/Sensors");
    auto perception_node = rmw_create_node(test_context(), "Perception", "/ADAS");

    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();

    EXPECT_RMW_OK(rmw_get_node_names(test_node(), &names, &namespaces));

    ASSERT_GE(names.size, 3u);
    ASSERT_GE(namespaces.size, 3u);
    ASSERT_TRUE(contains_node_name_and_namespace("Camera", "/Sensors", names, namespaces));
    ASSERT_TRUE(contains_node_name_and_namespace("Lidar", "/Sensors", names, namespaces));
    ASSERT_TRUE(contains_node_name_and_namespace("Perception", "/ADAS", names, namespaces));

    ASSERT_RMW_OK(rcutils_string_array_fini(&names));
    ASSERT_RMW_OK(rcutils_string_array_fini(&namespaces));

    ASSERT_RMW_OK(rmw_destroy_node(camera_node));
    ASSERT_RMW_OK(rmw_destroy_node(lidar_node));
    ASSERT_RMW_OK(rmw_destroy_node(perception_node));
}

TEST_F(RmwGraphTest, can_count_publishers) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto test_topic_a = create_test_topic("/TopicA");
    auto test_topic_b = create_test_topic("/TopicB");
    auto test_topic_c = create_test_topic("/TopicC");
    create_default_publisher<Defaults>(test_topic_a.c_str());
    create_default_publisher<Defaults>(test_topic_b.c_str());
    create_default_publisher<Defaults>(test_topic_c.c_str());

    // TODO: Add support to iceoryx2_cxx to get number of publishers
    // size_t count{0};
    // ASSERT_RMW_OK(rmw_count_publishers(test_node(), "Defaults", &count));
    // ASSERT_EQ(count, 3);
}

TEST_F(RmwGraphTest, can_count_subscribers) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto test_topic_a = create_test_topic("/TopicA");
    auto test_topic_b = create_test_topic("/TopicB");
    auto test_topic_c = create_test_topic("/TopicC");
    create_default_publisher<Defaults>(test_topic_a.c_str());
    create_default_publisher<Defaults>(test_topic_b.c_str());
    create_default_publisher<Defaults>(test_topic_c.c_str());

    // TODO: Add support to iceoryx2_cxx to get number of subscribers
    // size_t count{0};
    // ASSERT_RMW_OK(rmw_count_subscribers(test_node(), "Defaults", &count));
    // ASSERT_EQ(count, 3);
}

TEST_F(RmwGraphTest, can_get_topic_names_and_types) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto test_topic_a = create_test_topic("/TopicA");
    auto test_topic_b = create_test_topic("/TopicB");
    auto test_topic_c = create_test_topic("/TopicC");
    create_default_publisher<Defaults>(test_topic_a.c_str());
    create_default_publisher<Defaults>(test_topic_b.c_str());
    create_default_publisher<Defaults>(test_topic_c.c_str());
    create_default_subscriber<Defaults>(test_topic_a.c_str());
    create_default_subscriber<Defaults>(test_topic_b.c_str());
    create_default_subscriber<Defaults>(test_topic_c.c_str());

    auto allocator = rcutils_get_default_allocator();
    auto topic_names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_names_and_types_init(&topic_names_and_types, 0, &allocator));
    ASSERT_RMW_OK(rmw_get_topic_names_and_types(test_node(), &allocator, false, &topic_names_and_types));

    for (size_t i = 0; i < topic_names_and_types.names.size; i++) {
        printf("Topic name %zu: %s\n", i, topic_names_and_types.names.data[i]);
    }
    // ASSERT_EQ(topic_names_and_types.names.size, 3); // Needs domain isolation
    ASSERT_TRUE(rcutils_string_array_contains(&topic_names_and_types.names, test_topic_a.c_str()));
    ASSERT_TRUE(rcutils_string_array_contains(&topic_names_and_types.names, test_topic_b.c_str()));
    ASSERT_TRUE(rcutils_string_array_contains(&topic_names_and_types.names, test_topic_c.c_str()));

    // TODO: Make it possible to get typename from iceoryx2 service
    //       Requires capability to store ROS typename in iceoryx2 service attributes
    //       Currently available in Rust but not CXX
    // ASSERT_EQ(topic_names_and_types.types->size, 3); // Needs domain isolation
    ASSERT_TRUE(rcutils_string_array_contains(topic_names_and_types.types, "UNKNOWN"));
    ASSERT_TRUE(rcutils_string_array_contains(topic_names_and_types.types, "UNKNOWN"));
    ASSERT_TRUE(rcutils_string_array_contains(topic_names_and_types.types, "UNKNOWN"));

    ASSERT_RMW_OK(rmw_names_and_types_fini(&topic_names_and_types));
}

} // namespace
