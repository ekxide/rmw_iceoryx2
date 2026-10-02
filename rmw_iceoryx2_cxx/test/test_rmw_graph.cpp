// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/node.hpp"
#include "iox2/service.hpp"
#include "iox2/service_builder_publish_subscribe.hpp"
#include "iox2/service_name.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/strdup.h"
#include "rmw/get_node_info_and_types.h"
#include "rmw/get_service_endpoint_info.h"
#include "rmw/get_service_names_and_types.h"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/get_topic_names_and_types.h"
#include "rmw/names_and_types.h"
#include "rmw/rmw.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/graph.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/publisher.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

namespace
{

using namespace rmw::iox2::testing;

struct FailingAllocatorState
{
    size_t remaining;
    size_t live;
};

auto failing_allocate(size_t size, void* state) -> void* {
    auto* counts = static_cast<FailingAllocatorState*>(state);
    if (counts->remaining == 0) {
        return nullptr;
    }
    --counts->remaining;
    ++counts->live;
    return std::malloc(size);
}

auto failing_zero_allocate(size_t count, size_t size, void* state) -> void* {
    auto* counts = static_cast<FailingAllocatorState*>(state);
    if (counts->remaining == 0) {
        return nullptr;
    }
    --counts->remaining;
    ++counts->live;
    return std::calloc(count, size);
}

auto failing_reallocate(void* pointer, size_t size, void* state) -> void* {
    if (pointer == nullptr) {
        return failing_allocate(size, state);
    }
    return std::realloc(pointer, size);
}

auto failing_deallocate(void* pointer, void* state) -> void {
    if (pointer != nullptr) {
        --static_cast<FailingAllocatorState*>(state)->live;
        std::free(pointer);
    }
}

auto failing_allocator(FailingAllocatorState* state) -> rcutils_allocator_t {
    auto allocator = rcutils_get_zero_initialized_allocator();
    allocator.allocate = failing_allocate;
    allocator.deallocate = failing_deallocate;
    allocator.reallocate = failing_reallocate;
    allocator.zero_allocate = failing_zero_allocate;
    allocator.state = state;
    return allocator;
}

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
        if (!array || !str) {
            return false;
        }
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

// ---------------------------------------------------------------------------
// Node names
// ---------------------------------------------------------------------------

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

TEST_F(RmwGraphTest, lists_every_node_that_shares_a_name) {
    auto first = rmw_create_node(test_context(), "Twin", "/Sensors");
    auto second = rmw_create_node(test_context(), "Twin", "/Sensors");
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);

    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    EXPECT_RMW_OK(rmw_get_node_names(test_node(), &names, &namespaces));

    size_t twins = 0;
    for (size_t i = 0; i < names.size; ++i) {
        if (strcmp(names.data[i], "Twin") == 0 && strcmp(namespaces.data[i], "/Sensors") == 0) {
            ++twins;
        }
    }
    EXPECT_EQ(twins, 2U);

    ASSERT_RMW_OK(rcutils_string_array_fini(&names));
    ASSERT_RMW_OK(rcutils_string_array_fini(&namespaces));
    ASSERT_RMW_OK(rmw_destroy_node(first));
    ASSERT_RMW_OK(rmw_destroy_node(second));
}

TEST_F(RmwGraphTest, accepts_string_arrays_that_were_finalized_before) {
    auto allocator = rcutils_get_default_allocator();
    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    ASSERT_RMW_OK(rcutils_string_array_init(&names, 1, &allocator));
    ASSERT_RMW_OK(rcutils_string_array_fini(&names));

    EXPECT_RMW_OK(rmw_get_node_names(test_node(), &names, &namespaces));
    EXPECT_GE(names.size, 1U);

    ASSERT_RMW_OK(rcutils_string_array_fini(&names));
    ASSERT_RMW_OK(rcutils_string_array_fini(&namespaces));
}

TEST_F(RmwGraphTest, can_get_node_names_with_enclaves) {
    auto camera_node = rmw_create_node(test_context(), "Camera", "/Sensors");
    ASSERT_NE(camera_node, nullptr);

    auto options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&options, test_allocator()));
    test_allocator().deallocate(options.enclave, test_allocator().state);
    options.enclave = rcutils_strdup("/sensors", test_allocator());
    auto context = rmw_get_zero_initialized_context();
    ASSERT_RMW_OK(rmw_init(&options, &context));
    auto lidar_node = rmw_create_node(&context, "Lidar", "/Sensors");
    ASSERT_NE(lidar_node, nullptr);

    rcutils_string_array_t names = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t namespaces = rcutils_get_zero_initialized_string_array();
    rcutils_string_array_t enclaves = rcutils_get_zero_initialized_string_array();
    EXPECT_RMW_OK(rmw_get_node_names_with_enclaves(test_node(), &names, &namespaces, &enclaves));

    ASSERT_EQ(enclaves.size, names.size);
    auto enclave_of = [&](const char* name) -> std::string {
        for (size_t i = 0; i < names.size; ++i) {
            if (strcmp(names.data[i], name) == 0 && strcmp(namespaces.data[i], "/Sensors") == 0) {
                return enclaves.data[i];
            }
        }
        return "not listed";
    };
    EXPECT_EQ(enclave_of("Camera"), "/");
    EXPECT_EQ(enclave_of("Lidar"), "/sensors");

    ASSERT_RMW_OK(rcutils_string_array_fini(&names));
    ASSERT_RMW_OK(rcutils_string_array_fini(&namespaces));
    ASSERT_RMW_OK(rcutils_string_array_fini(&enclaves));
    ASSERT_RMW_OK(rmw_destroy_node(lidar_node));
    ASSERT_RMW_OK(rmw_shutdown(&context));
    ASSERT_RMW_OK(rmw_context_fini(&context));
    ASSERT_RMW_OK(rmw_init_options_fini(&options));
    ASSERT_RMW_OK(rmw_destroy_node(camera_node));
}

// ---------------------------------------------------------------------------
// Endpoint counts
// ---------------------------------------------------------------------------

TEST_F(RmwGraphTest, can_count_publishers) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/CountPublishers");
    create_default_publisher<Defaults>(topic);
    create_default_publisher<Defaults>(topic);

    size_t count{0};
    ASSERT_RMW_OK(rmw_count_publishers(test_node(), topic.c_str(), &count));
    ASSERT_EQ(count, 2u);
}

TEST_F(RmwGraphTest, can_count_subscribers) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/CountSubscribers");
    create_default_subscriber<Defaults>(topic);
    create_default_subscriber<Defaults>(topic);
    create_default_subscriber<Defaults>(topic);

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

// ---------------------------------------------------------------------------
// Endpoint info
// ---------------------------------------------------------------------------

TEST_F(RmwGraphTest, can_get_publishers_info_by_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/PublishersInfo");
    create_default_publisher<Defaults>(topic);

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

TEST_F(RmwGraphTest, names_publishers_of_unknown_nodes_as_unknown) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    using Payload = rmw::iox2::Publisher::Payload;
    using UserHeader = rmw::iox2::Publisher::UserHeader;

    auto topic = create_test_topic("/SharedWithNative");
    ASSERT_NE(create_default_publisher<Defaults>(topic), nullptr);

    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto service_name = iox2::ServiceName::create(("ros2://topics" + topic).c_str()).value();
    auto details = iox2::Service<iox2::ServiceType::Ipc>::details(
        service_name, native_node.config(), iox2::MessagingPattern::PublishSubscribe);
    ASSERT_TRUE(details.has_value());
    ASSERT_TRUE(details.value().has_value());
    auto builder = native_node.service_builder(service_name).publish_subscribe<Payload>().user_header<UserHeader>();
    iox2::set_payload_type_details(
        builder, details.value().value().static_details.publish_subscribe().message_type_details().payload());
    auto native_service = builder.resume_build().open();
    ASSERT_TRUE(native_service.has_value());
    auto native_publisher = native_service.value().publisher_builder().create();
    ASSERT_TRUE(native_publisher.has_value());

    auto allocator = rcutils_get_default_allocator();
    auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
    ASSERT_RMW_OK(rmw_get_publishers_info_by_topic(test_node(), &allocator, topic.c_str(), false, &info));
    ASSERT_EQ(info.size, 2U);
    size_t unknown = 0;
    for (size_t i = 0; i < info.size; ++i) {
        if (std::string(info.info_array[i].node_name) == rmw::iox2::UNKNOWN_NODE_NAME) {
            EXPECT_STREQ(info.info_array[i].node_namespace, rmw::iox2::UNKNOWN_NODE_NAMESPACE);
            ++unknown;
        }
    }
    EXPECT_EQ(unknown, 1U);

    ASSERT_RMW_OK(rmw_topic_endpoint_info_array_fini(&info, &allocator));
}

TEST_F(RmwGraphTest, can_get_subscriptions_info_by_topic) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/SubscriptionsInfo");
    create_default_subscriber<Defaults>(topic);

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

// ---------------------------------------------------------------------------
// Topics
// ---------------------------------------------------------------------------

TEST_F(RmwGraphTest, can_get_topic_names_and_types) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto test_topic_a = create_test_topic("/TopicA");
    auto test_topic_b = create_test_topic("/TopicB");
    auto test_topic_c = create_test_topic("/TopicC");
    create_default_publisher<Defaults>(test_topic_a);
    create_default_publisher<Defaults>(test_topic_b);
    create_default_publisher<Defaults>(test_topic_c);
    create_default_subscriber<Defaults>(test_topic_a);
    create_default_subscriber<Defaults>(test_topic_b);
    create_default_subscriber<Defaults>(test_topic_c);

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

TEST_F(RmwGraphTest, reports_no_services_or_clients) {
    auto allocator = rcutils_get_default_allocator();
    const auto* node_name = test_node()->name;
    const auto* node_namespace = test_node()->namespace_;

    size_t count{1};
    EXPECT_RMW_OK(rmw_count_services(test_node(), "/service", &count));
    EXPECT_EQ(count, 0U);
    count = 1;
    EXPECT_RMW_OK(rmw_count_clients(test_node(), "/service", &count));
    EXPECT_EQ(count, 0U);

    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    EXPECT_RMW_OK(rmw_get_service_names_and_types(test_node(), &allocator, &names_and_types));
    EXPECT_EQ(names_and_types.names.size, 0U);
    EXPECT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
    EXPECT_RMW_OK(
        rmw_get_service_names_and_types_by_node(test_node(), &allocator, node_name, node_namespace, &names_and_types));
    EXPECT_EQ(names_and_types.names.size, 0U);
    EXPECT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
    EXPECT_RMW_OK(
        rmw_get_client_names_and_types_by_node(test_node(), &allocator, node_name, node_namespace, &names_and_types));
    EXPECT_EQ(names_and_types.names.size, 0U);
    EXPECT_RMW_OK(rmw_names_and_types_fini(&names_and_types));

    auto servers_info = rmw_get_zero_initialized_service_endpoint_info_array();
    EXPECT_RMW_OK(rmw_get_servers_info_by_service(test_node(), &allocator, "/service", false, &servers_info));
    EXPECT_EQ(servers_info.size, 0U);
    auto clients_info = rmw_get_zero_initialized_service_endpoint_info_array();
    EXPECT_RMW_OK(rmw_get_clients_info_by_service(test_node(), &allocator, "/service", false, &clients_info));
    EXPECT_EQ(clients_info.size, 0U);
}

TEST_F(RmwGraphTest, rejects_non_zero_initialized_names_and_types) {
    auto allocator = rcutils_get_default_allocator();
    auto topic_names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_names_and_types_init(&topic_names_and_types, 1, &allocator));

    EXPECT_RMW_ERR(RMW_RET_INVALID_ARGUMENT,
                   rmw_get_topic_names_and_types(test_node(), &allocator, false, &topic_names_and_types));

    ASSERT_RMW_OK(rmw_names_and_types_fini(&topic_names_and_types));
}

// ---------------------------------------------------------------------------
// By node
// ---------------------------------------------------------------------------

TEST_F(RmwGraphTest, can_get_publisher_names_and_types_by_node) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto published_topic = create_test_topic("/PublishedByNode");
    auto subscribed_topic = create_test_topic("/SubscribedByNode");
    create_default_publisher<Defaults>(published_topic);
    create_default_subscriber<Defaults>(subscribed_topic);

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

TEST_F(RmwGraphTest, names_and_types_by_node_skip_topics_that_cannot_be_opened) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto published_topic = create_test_topic("/PublishedByNode");
    create_default_publisher<Defaults>(published_topic);

    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service_name = "ros2://topics" + create_test_topic("/Native");
    auto native_service = native_node.service_builder(iox2::ServiceName::create(native_service_name.c_str()).value())
                              .publish_subscribe<uint64_t>()
                              .create();
    ASSERT_TRUE(native_service.has_value());

    auto allocator = rcutils_get_default_allocator();
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    ASSERT_RMW_OK(rmw_get_publisher_names_and_types_by_node(
        test_node(), &allocator, test_node()->name, test_node()->namespace_, false, &names_and_types));
    EXPECT_STREQ(first_type_of_topic(names_and_types, published_topic.c_str()),
                 "rmw_iceoryx2_cxx_test_msgs/msg/Defaults");
    EXPECT_FALSE(rcutils_error_is_set());

    ASSERT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
}

TEST_F(RmwGraphTest, can_get_subscriber_names_and_types_by_node) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto subscribed_topic = create_test_topic("/SubscribedByNode");
    auto published_topic = create_test_topic("/PublishedByNode");
    create_default_subscriber<Defaults>(subscribed_topic);
    create_default_publisher<Defaults>(published_topic);

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

TEST_F(RmwGraphTest, names_and_types_by_node_of_an_unknown_node_do_not_exist) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto topic = create_test_topic("/OwnedByTestNode");
    create_default_publisher<Defaults>(topic);

    auto allocator = rcutils_get_default_allocator();
    auto names_and_types = rmw_get_zero_initialized_names_and_types();
    const auto* node_namespace = test_node()->namespace_;
    EXPECT_RMW_ERR(RMW_RET_NODE_NAME_NON_EXISTENT,
                   rmw_get_publisher_names_and_types_by_node(
                       test_node(), &allocator, "UnknownNode", node_namespace, false, &names_and_types));
    EXPECT_RMW_ERR(RMW_RET_NODE_NAME_NON_EXISTENT,
                   rmw_get_subscriber_names_and_types_by_node(
                       test_node(), &allocator, "UnknownNode", node_namespace, false, &names_and_types));
    EXPECT_RMW_ERR(RMW_RET_NODE_NAME_NON_EXISTENT,
                   rmw_get_service_names_and_types_by_node(
                       test_node(), &allocator, "UnknownNode", node_namespace, &names_and_types));
    EXPECT_RMW_ERR(RMW_RET_NODE_NAME_NON_EXISTENT,
                   rmw_get_client_names_and_types_by_node(
                       test_node(), &allocator, "UnknownNode", node_namespace, &names_and_types));
    EXPECT_RMW_OK(rmw_names_and_types_check_zero(&names_and_types));
}

TEST_F(RmwGraphTest, frees_topic_names_and_types_when_an_allocation_fails) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    create_default_publisher<Defaults>(create_test_topic("/First"));
    create_default_publisher<Defaults>(create_test_topic("/Second"));

    auto ret = RMW_RET_BAD_ALLOC;
    for (size_t allowed = 0; ret == RMW_RET_BAD_ALLOC && allowed < 1000; ++allowed) {
        FailingAllocatorState state{allowed, 0};
        auto allocator = failing_allocator(&state);
        auto names_and_types = rmw_get_zero_initialized_names_and_types();
        ret = rmw_get_topic_names_and_types(test_node(), &allocator, false, &names_and_types);
        if (ret == RMW_RET_OK) {
            EXPECT_RMW_OK(rmw_names_and_types_fini(&names_and_types));
        } else {
            EXPECT_EQ(names_and_types.names.data, nullptr);
            EXPECT_EQ(names_and_types.types, nullptr);
        }
        EXPECT_EQ(state.live, 0U);
        rcutils_reset_error();
    }
    EXPECT_RMW_OK(ret);
}

TEST_F(RmwGraphTest, frees_endpoint_info_when_an_allocation_fails) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;
    auto topic = create_test_topic("/Endpoints");
    create_default_publisher<Defaults>(topic);
    create_default_publisher<Defaults>(topic);

    auto ret = RMW_RET_BAD_ALLOC;
    for (size_t allowed = 0; ret == RMW_RET_BAD_ALLOC && allowed < 1000; ++allowed) {
        FailingAllocatorState state{allowed, 0};
        auto allocator = failing_allocator(&state);
        auto info = rmw_get_zero_initialized_topic_endpoint_info_array();
        ret = rmw_get_publishers_info_by_topic(test_node(), &allocator, topic.c_str(), false, &info);
        if (ret == RMW_RET_OK) {
            EXPECT_RMW_OK(rmw_topic_endpoint_info_array_fini(&info, &allocator));
        } else {
            EXPECT_RMW_OK(rmw_topic_endpoint_info_array_check_zero(&info));
        }
        EXPECT_EQ(state.live, 0U);
        rcutils_reset_error();
    }
    EXPECT_RMW_OK(ret);
}

TEST_F(RmwGraphTest, graph_guard_condition_is_triggered_when_another_context_changes_the_graph) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    const auto* graph_guard_condition = rmw_node_get_graph_guard_condition(test_node());
    ASSERT_NE(graph_guard_condition, nullptr);
    auto* waitset = rmw_create_wait_set(test_context(), 1);
    ASSERT_NE(waitset, nullptr);
    void* conditions[] = {graph_guard_condition->data};
    rmw_guard_conditions_t guard_conditions{1, conditions};
    auto wait = [&](rmw_time_t timeout) {
        guard_conditions.guard_conditions[0] = graph_guard_condition->data;
        return rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, waitset, &timeout);
    };

    (void)wait(rmw_time_t{0, 0});
    EXPECT_EQ(wait(rmw_time_t{0, 20000000}), RMW_RET_TIMEOUT);

    auto options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&options, test_allocator()));
    auto context = rmw_get_zero_initialized_context();
    ASSERT_RMW_OK(rmw_init(&options, &context));
    auto* node = rmw_create_node(&context, "Other", "/Sensors");
    ASSERT_NE(node, nullptr);
    EXPECT_RMW_OK(wait(rmw_time_t{1, 0}));
    EXPECT_NE(guard_conditions.guard_conditions[0], nullptr);

    auto topic = create_test_topic("/GraphChange");
    auto qos = rmw_qos_profile_default;
    auto publisher_options = rmw_get_default_publisher_options();
    auto* publisher =
        rmw_create_publisher(node, test_type_support<Defaults>(), topic.c_str(), &qos, &publisher_options);
    ASSERT_NE(publisher, nullptr);
    EXPECT_RMW_OK(wait(rmw_time_t{1, 0}));
    EXPECT_NE(guard_conditions.guard_conditions[0], nullptr);

    ASSERT_RMW_OK(rmw_destroy_publisher(node, publisher));
    EXPECT_RMW_OK(wait(rmw_time_t{1, 0}));
    EXPECT_NE(guard_conditions.guard_conditions[0], nullptr);

    ASSERT_RMW_OK(rmw_destroy_node(node));
    EXPECT_RMW_OK(wait(rmw_time_t{1, 0}));
    EXPECT_NE(guard_conditions.guard_conditions[0], nullptr);

    ASSERT_RMW_OK(rmw_shutdown(&context));
    ASSERT_RMW_OK(rmw_context_fini(&context));
    ASSERT_RMW_OK(rmw_init_options_fini(&options));
    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
}

TEST_F(RmwGraphTest, graph_guard_conditions_of_every_node_in_a_wait_set_are_triggered) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    auto* other_node = create_default_node("OtherObserver");
    ASSERT_NE(other_node, nullptr);
    auto* waitset = rmw_create_wait_set(test_context(), 2);
    ASSERT_NE(waitset, nullptr);
    void* first = rmw_node_get_graph_guard_condition(test_node())->data;
    void* second = rmw_node_get_graph_guard_condition(other_node)->data;
    void* conditions[] = {first, second};
    rmw_guard_conditions_t guard_conditions{2, conditions};
    auto wait = [&](rmw_time_t timeout) {
        guard_conditions.guard_conditions[0] = first;
        guard_conditions.guard_conditions[1] = second;
        return rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, waitset, &timeout);
    };
    (void)wait(rmw_time_t{0, 0});

    create_default_publisher<Defaults>(create_test_topic("/GraphChange"));
    EXPECT_RMW_OK(wait(rmw_time_t{1, 0}));
    EXPECT_NE(guard_conditions.guard_conditions[0], nullptr);
    EXPECT_NE(guard_conditions.guard_conditions[1], nullptr);

    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
}

TEST_F(RmwGraphTest, triggering_a_graph_guard_condition_wakes_a_waiting_wait_set) {
    const auto* graph_guard_condition = rmw_node_get_graph_guard_condition(test_node());
    ASSERT_NE(graph_guard_condition, nullptr);
    auto* waitset = rmw_create_wait_set(test_context(), 1);
    ASSERT_NE(waitset, nullptr);
    void* conditions[] = {graph_guard_condition->data};
    rmw_guard_conditions_t guard_conditions{1, conditions};
    rmw_time_t no_wait{0, 0};
    EXPECT_NE(rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, waitset, &no_wait), RMW_RET_ERROR);

    std::thread trigger([graph_guard_condition] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        EXPECT_RMW_OK(rmw_trigger_guard_condition(graph_guard_condition));
    });
    guard_conditions.guard_conditions[0] = graph_guard_condition->data;
    rmw_time_t timeout{5, 0};
    auto start = std::chrono::steady_clock::now();
    EXPECT_RMW_OK(rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, waitset, &timeout));
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
    EXPECT_NE(guard_conditions.guard_conditions[0], nullptr);
    trigger.join();

    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
}

class RmwGraphServiceTest : public TestBase
{
protected:
    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(RmwGraphServiceTest, contexts_join_a_graph_service_created_by_another_iceoryx2_application) {
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service =
        native_node.service_builder(iox2::ServiceName::create(rmw::iox2::names::graph().c_str()).value())
            .event()
            .max_nodes(2)
            .max_notifiers(1)
            .max_listeners(1)
            .create();
    ASSERT_TRUE(native_service.has_value());

    auto options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&options, test_allocator()));
    auto context = rmw_get_zero_initialized_context();
    ASSERT_RMW_OK(rmw_init(&options, &context));
    EXPECT_EQ(native_service->dynamic_config().number_of_notifiers(), 1U);

    auto* observer = rmw_create_node(&context, "Observer", "/Sensors");
    ASSERT_NE(observer, nullptr);
    auto* waitset = rmw_create_wait_set(&context, 1);
    ASSERT_NE(waitset, nullptr);
    void* conditions[] = {rmw_node_get_graph_guard_condition(observer)->data};
    rmw_guard_conditions_t guard_conditions{1, conditions};
    rmw_time_t timeout{0, 20000000};
    auto result = rmw_wait(nullptr, &guard_conditions, nullptr, nullptr, nullptr, waitset, &timeout);
    EXPECT_TRUE(result == RMW_RET_OK || result == RMW_RET_TIMEOUT) << "rmw_wait returned " << result;
    EXPECT_FALSE(rcutils_error_is_set());

    ASSERT_RMW_OK(rmw_destroy_wait_set(waitset));
    ASSERT_RMW_OK(rmw_destroy_node(observer));
    ASSERT_RMW_OK(rmw_shutdown(&context));
    ASSERT_RMW_OK(rmw_context_fini(&context));
    ASSERT_RMW_OK(rmw_init_options_fini(&options));
}

TEST_F(RmwGraphServiceTest, init_fails_when_the_graph_service_has_no_room_for_another_context) {
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service =
        native_node.service_builder(iox2::ServiceName::create(rmw::iox2::names::graph().c_str()).value())
            .event()
            .max_notifiers(1)
            .create();
    ASSERT_TRUE(native_service.has_value());
    auto native_notifier = native_service->notifier_builder().create();
    ASSERT_TRUE(native_notifier.has_value());

    auto options = rmw_get_zero_initialized_init_options();
    ASSERT_RMW_OK(rmw_init_options_init(&options, test_allocator()));
    auto context = rmw_get_zero_initialized_context();
    EXPECT_EQ(rmw_init(&options, &context), RMW_RET_ERROR);
    rcutils_reset_error();

    ASSERT_RMW_OK(rmw_init_options_fini(&options));
}

} // namespace
