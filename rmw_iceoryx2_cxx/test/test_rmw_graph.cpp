// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "rmw/get_node_info_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"
#include "rmw/topic_endpoint_info_array.h"
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

    // Returns the first type registered for `topic`, or nullptr if the topic is absent.
    const char* first_type_of_topic(const rmw_names_and_types_t& names_and_types, const char* topic) {
        for (size_t i = 0; i < names_and_types.names.size; ++i) {
            if (strcmp(names_and_types.names.data[i], topic) == 0) {
                return names_and_types.types[i].size > 0 ? names_and_types.types[i].data[0] : nullptr;
            }
        }
        return nullptr;
    }

    bool gid_is_nonzero(const uint8_t (&gid)[RMW_GID_STORAGE_SIZE]) {
        for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
            if (gid[i] != 0) {
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

    auto topic = create_test_topic("/CountPublishers");
    create_default_publisher<Defaults>(topic.c_str());
    create_default_publisher<Defaults>(topic.c_str());

    size_t count{0};
    ASSERT_RMW_OK(rmw_count_publishers(test_node(), topic.c_str(), &count));
    ASSERT_EQ(count, 2u);
}

TEST_F(RmwGraphTest, can_count_subscribers) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/CountSubscribers");
    create_default_subscriber<Defaults>(topic.c_str());
    create_default_subscriber<Defaults>(topic.c_str());
    create_default_subscriber<Defaults>(topic.c_str());

    size_t count{0};
    ASSERT_RMW_OK(rmw_count_subscribers(test_node(), topic.c_str(), &count));
    ASSERT_EQ(count, 3u);
}

TEST_F(RmwGraphTest, counts_zero_endpoints_for_unknown_topic) {
    auto topic = create_test_topic("/NoEndpoints");

    size_t publishers{99};
    size_t subscribers{99};
    ASSERT_RMW_OK(rmw_count_publishers(test_node(), topic.c_str(), &publishers));
    ASSERT_RMW_OK(rmw_count_subscribers(test_node(), topic.c_str(), &subscribers));
    ASSERT_EQ(publishers, 0u);
    ASSERT_EQ(subscribers, 0u);
}

TEST_F(RmwGraphTest, can_get_publishers_info_by_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/PublishersInfo");
    create_default_publisher<Defaults>(topic.c_str());

    auto allocator = rcutils_get_default_allocator();
    auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
    ASSERT_RMW_OK(rmw_get_publishers_info_by_topic(test_node(), &allocator, topic.c_str(), false, &info));

    ASSERT_EQ(info.size, 1u);
    const auto& endpoint = info.info_array[0];
    EXPECT_EQ(endpoint.endpoint_type, RMW_ENDPOINT_PUBLISHER);
    EXPECT_STREQ(endpoint.topic_type, "rmw_iceoryx2_cxx_test_msgs/msg/Defaults");
    EXPECT_NE(endpoint.topic_type_hash.version, ROSIDL_TYPE_HASH_VERSION_UNSET);
    EXPECT_STREQ(endpoint.node_namespace, "/RmwTest");
    EXPECT_GT(strlen(endpoint.node_name), 0u);
    EXPECT_TRUE(gid_is_nonzero(endpoint.endpoint_gid));

    ASSERT_RMW_OK(rmw_topic_endpoint_info_array_fini(&info, &allocator));
}

TEST_F(RmwGraphTest, can_get_subscriptions_info_by_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/SubscriptionsInfo");
    create_default_subscriber<Defaults>(topic.c_str());

    auto allocator = rcutils_get_default_allocator();
    auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
    ASSERT_RMW_OK(rmw_get_subscriptions_info_by_topic(test_node(), &allocator, topic.c_str(), false, &info));

    ASSERT_EQ(info.size, 1u);
    const auto& endpoint = info.info_array[0];
    EXPECT_EQ(endpoint.endpoint_type, RMW_ENDPOINT_SUBSCRIPTION);
    EXPECT_STREQ(endpoint.topic_type, "rmw_iceoryx2_cxx_test_msgs/msg/Defaults");
    EXPECT_NE(endpoint.topic_type_hash.version, ROSIDL_TYPE_HASH_VERSION_UNSET);
    EXPECT_STREQ(endpoint.node_namespace, "/RmwTest");
    EXPECT_GT(strlen(endpoint.node_name), 0u);
    EXPECT_TRUE(gid_is_nonzero(endpoint.endpoint_gid));

    ASSERT_RMW_OK(rmw_topic_endpoint_info_array_fini(&info, &allocator));
}

TEST_F(RmwGraphTest, gets_empty_endpoint_info_for_unknown_topic) {
    auto topic = create_test_topic("/NoEndpointInfo");

    auto allocator = rcutils_get_default_allocator();
    auto publishers = rmw_get_zero_initialized_topic_endpoint_info_array();
    auto subscriptions = rmw_get_zero_initialized_topic_endpoint_info_array();
    ASSERT_RMW_OK(rmw_get_publishers_info_by_topic(test_node(), &allocator, topic.c_str(), false, &publishers));
    ASSERT_RMW_OK(rmw_get_subscriptions_info_by_topic(test_node(), &allocator, topic.c_str(), false, &subscriptions));

    EXPECT_EQ(publishers.size, 0u);
    EXPECT_EQ(subscriptions.size, 0u);

    ASSERT_RMW_OK(rmw_topic_endpoint_info_array_fini(&publishers, &allocator));
    ASSERT_RMW_OK(rmw_topic_endpoint_info_array_fini(&subscriptions, &allocator));
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
    ASSERT_RMW_OK(rmw_get_topic_names_and_types(test_node(), &allocator, false, &topic_names_and_types));

    // ASSERT_EQ(topic_names_and_types.names.size, 3); // Needs domain isolation
    ASSERT_TRUE(rcutils_string_array_contains(&topic_names_and_types.names, test_topic_a.c_str()));
    ASSERT_TRUE(rcutils_string_array_contains(&topic_names_and_types.names, test_topic_b.c_str()));
    ASSERT_TRUE(rcutils_string_array_contains(&topic_names_and_types.names, test_topic_c.c_str()));

    // Each topic carries its own ROS type name in its own sub-array.
    const char* expected_type = "rmw_iceoryx2_cxx_test_msgs/msg/Defaults";
    EXPECT_STREQ(first_type_of_topic(topic_names_and_types, test_topic_a.c_str()), expected_type);
    EXPECT_STREQ(first_type_of_topic(topic_names_and_types, test_topic_b.c_str()), expected_type);
    EXPECT_STREQ(first_type_of_topic(topic_names_and_types, test_topic_c.c_str()), expected_type);

    ASSERT_RMW_OK(rmw_names_and_types_fini(&topic_names_and_types));
}

TEST_F(RmwGraphTest, accepts_zero_initialized_names_and_types) {
    // Callers such as `ros2 topic list` pass a zero-initialized
    // `rmw_names_and_types_t`, whose `types` member is NULL. The query must
    // accept that form.
    auto allocator = rcutils_get_default_allocator();
    auto topic_names_and_types = rmw_get_zero_initialized_names_and_types();

    EXPECT_RMW_OK(rmw_get_topic_names_and_types(test_node(), &allocator, false, &topic_names_and_types));

    ASSERT_RMW_OK(rmw_names_and_types_fini(&topic_names_and_types));
}

TEST_F(RmwGraphTest, can_get_publisher_names_and_types_by_node) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto published_topic = create_test_topic("/PublishedByNode");
    auto subscribed_topic = create_test_topic("/SubscribedByNode");
    create_default_publisher<Defaults>(published_topic.c_str());
    create_default_subscriber<Defaults>(subscribed_topic.c_str());

    auto allocator = rcutils_get_default_allocator();
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_get_publisher_names_and_types_by_node(
        test_node(), &allocator, test_node()->name, test_node()->namespace_, false, &names_and_types));

    // The node's published topic is listed with its type.
    EXPECT_STREQ(first_type_of_topic(names_and_types, published_topic.c_str()),
                 "rmw_iceoryx2_cxx_test_msgs/msg/Defaults");
    // A topic the node only subscribes to is not a publisher of the node.
    EXPECT_FALSE(rcutils_string_array_contains(&names_and_types.names, subscribed_topic.c_str()));

    ASSERT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
}

TEST_F(RmwGraphTest, can_get_subscriber_names_and_types_by_node) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto subscribed_topic = create_test_topic("/SubscribedByNode");
    auto published_topic = create_test_topic("/PublishedByNode");
    create_default_subscriber<Defaults>(subscribed_topic.c_str());
    create_default_publisher<Defaults>(published_topic.c_str());

    auto allocator = rcutils_get_default_allocator();
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_get_subscriber_names_and_types_by_node(
        test_node(), &allocator, test_node()->name, test_node()->namespace_, false, &names_and_types));

    // The node's subscribed topic is listed with its type.
    EXPECT_STREQ(first_type_of_topic(names_and_types, subscribed_topic.c_str()),
                 "rmw_iceoryx2_cxx_test_msgs/msg/Defaults");
    // A topic the node only publishes to is not a subscriber of the node.
    EXPECT_FALSE(rcutils_string_array_contains(&names_and_types.names, published_topic.c_str()));

    ASSERT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
}

TEST_F(RmwGraphTest, gets_empty_names_and_types_for_unknown_node) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/OwnedByTestNode");
    create_default_publisher<Defaults>(topic.c_str());

    auto allocator = rcutils_get_default_allocator();
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_get_publisher_names_and_types_by_node(
        test_node(), &allocator, "UnknownNode", test_node()->namespace_, false, &names_and_types));

    EXPECT_EQ(names_and_types.names.size, 0u);

    ASSERT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
}

TEST_F(RmwGraphTest, can_get_node_names_with_enclaves) {
    auto camera_node = rmw_create_node(test_context(), "Camera", "/Sensors");

    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t enclaves = rcutils_get_zero_initialized_string_array();

    EXPECT_RMW_OK(rmw_get_node_names_with_enclaves(test_node(), &names, &namespaces, &enclaves));

    ASSERT_TRUE(contains_node_name_and_namespace("Camera", "/Sensors", names, namespaces));
    // Names, namespaces and enclaves are parallel arrays; every node reports the
    // default enclave since the transport carries no SROS2 information.
    ASSERT_EQ(enclaves.size, names.size);
    for (size_t i = 0; i < enclaves.size; ++i) {
        EXPECT_STREQ(enclaves.data[i], "/");
    }

    ASSERT_RMW_OK(rcutils_string_array_fini(&names));
    ASSERT_RMW_OK(rcutils_string_array_fini(&namespaces));
    ASSERT_RMW_OK(rcutils_string_array_fini(&enclaves));

    ASSERT_RMW_OK(rmw_destroy_node(camera_node));
}

TEST_F(RmwGraphTest, rejects_non_zero_initialized_names_and_types) {
    auto allocator = rcutils_get_default_allocator();
    auto topic_names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_names_and_types_init(&topic_names_and_types, 1, &allocator));

    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT,
                   rmw_get_topic_names_and_types(test_node(), &allocator, false, &topic_names_and_types));

    ASSERT_RMW_OK(rmw_names_and_types_fini(&topic_names_and_types));
}

} // namespace
