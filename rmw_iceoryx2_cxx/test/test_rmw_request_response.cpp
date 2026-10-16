// Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "iox2/node.hpp"
#include "iox2/service_name.hpp"
#include "rcutils/error_handling.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/common/names.hpp"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx/impl/message/message_info_header.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/context.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/payload_layout.hpp"
#include "rmw_iceoryx2_cxx/impl/runtime/server.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/basic_types.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/self_contained.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/srv/self_contained_request.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <algorithm>
#include <cstring>
#include <thread>

namespace
{
using namespace rmw::iox2::testing;
using rmw_iceoryx2_cxx_test_msgs::srv::BasicTypes;

class RmwRequestResponseTest : public TestBase
{
protected:
    void SetUp() override {
        initialize();
    }

    void TearDown() override {
        cleanup();
        print_rmw_errors();
    }

    bool is_ready(rmw_service_t* service, rmw_time_t timeout) {
        void* services[1] = {service->data};
        rmw_services_t rmw_services{1U, services};
        wait(&rmw_services, nullptr, timeout);
        return services[0] != nullptr;
    }

    bool is_ready(rmw_client_t* client, rmw_time_t timeout) {
        void* clients[1] = {client->data};
        rmw_clients_t rmw_clients{1U, clients};
        wait(nullptr, &rmw_clients, timeout);
        return clients[0] != nullptr;
    }

    void wait(rmw_services_t* services, rmw_clients_t* clients, rmw_time_t timeout) {
        auto* wait_set = rmw_create_wait_set(test_context(), 0);
        ASSERT_NE(wait_set, nullptr);
        auto result = rmw_wait(nullptr, nullptr, services, clients, nullptr, wait_set, &timeout);
        EXPECT_TRUE(result == RMW_RET_OK || result == RMW_RET_TIMEOUT);
        EXPECT_RMW_OK(rmw_destroy_wait_set(wait_set));
    }

    static auto open_native_self_contained_service(iox2::Node<iox2::ServiceType::Ipc>& native_node,
                                                   const std::string& topic) {
        using Server = ::rmw::iox2::Server;
        using rmw_iceoryx2_cxx_test_msgs::srv::SelfContained;

        const auto* type_support = rosidl_typesupport_cpp::get_service_type_support_handle<SelfContained>();
        const auto request_type_name = ::rmw::iox2::message_type_name(type_support->request_typesupport);
        const auto response_type_name = ::rmw::iox2::message_type_name(type_support->response_typesupport);
        auto builder =
            native_node
                .service_builder(iox2::ServiceName::create(::rmw::iox2::names::service(topic.c_str()).c_str()).value())
                .request_response<Server::Payload, Server::Payload>()
                .request_user_header<Server::UserHeader>()
                .response_user_header<Server::UserHeader>();
        iox2::set_request_payload_type_details(builder,
                                               iox2::TypeDetail(iox2::TypeVariant::FixedSize,
                                                                request_type_name.c_str(),
                                                                sizeof(SelfContained::Request),
                                                                ::rmw::iox2::SELF_CONTAINED_PAYLOAD_ALIGNMENT));
        iox2::set_response_payload_type_details(builder,
                                                iox2::TypeDetail(iox2::TypeVariant::FixedSize,
                                                                 response_type_name.c_str(),
                                                                 sizeof(SelfContained::Response),
                                                                 ::rmw::iox2::SELF_CONTAINED_PAYLOAD_ALIGNMENT));
        return builder.resume_build()
            .max_active_requests_per_client(::rmw::iox2::DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT)
            .max_response_buffer_size(::rmw::iox2::MAX_RESPONSES_PER_REQUEST)
            .request_payload_alignment(::rmw::iox2::SELF_CONTAINED_PAYLOAD_ALIGNMENT)
            .response_payload_alignment(::rmw::iox2::SELF_CONTAINED_PAYLOAD_ALIGNMENT)
            .open();
    }

    static BasicTypes::Request make_request(int32_t value) {
        BasicTypes::Request request;
        request.int32_value = value;
        request.string_value = "request " + std::to_string(value);
        return request;
    }

    static BasicTypes::Response make_response(int32_t value) {
        BasicTypes::Response response;
        response.int32_value = value;
        response.string_value = "response " + std::to_string(value);
        return response;
    }
};

TEST_F(RmwRequestResponseTest, a_request_reaches_the_server) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(42);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    BasicTypes::Request received_request;
    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &received_request, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_request, request);
    EXPECT_EQ(request_header.request_id.sequence_number, sequence_id);

    rmw_gid_t client_gid{};
    ASSERT_RMW_OK(rmw_get_gid_for_client(client, &client_gid));
    EXPECT_TRUE(std::equal(client_gid.data,
                           client_gid.data + RMW_GID_STORAGE_SIZE,
                           reinterpret_cast<const uint8_t*>(request_header.request_id.writer_guid)));

    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &received_request, &taken));
    EXPECT_FALSE(taken);
}

TEST_F(RmwRequestResponseTest, requests_are_numbered_in_order) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    int64_t first_id = 0;
    int64_t second_id = 0;
    auto first_request = make_request(1);
    auto second_request = make_request(2);
    ASSERT_RMW_OK(rmw_send_request(client, &first_request, &first_id));
    ASSERT_RMW_OK(rmw_send_request(client, &second_request, &second_id));
    EXPECT_LT(first_id, second_id);

    for (auto expected_id : {first_id, second_id}) {
        BasicTypes::Request request;
        rmw_service_info_t header{};
        bool taken = false;
        ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
        ASSERT_TRUE(taken);
        EXPECT_EQ(header.request_id.sequence_number, expected_id);
    }
}

TEST_F(RmwRequestResponseTest, requests_of_servers_that_are_gone_do_not_occupy_the_client) {
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(client, nullptr);

    for (size_t i = 0; i <= ::rmw::iox2::DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT; ++i) {
        auto* service = create_service<BasicTypes>(create_test_topic());
        RMW_ASSERT_NE(service, nullptr);

        auto request = make_request(static_cast<int32_t>(i));
        int64_t sequence_id = 0;
        ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
        destroy_service(service);
    }
}

TEST_F(RmwRequestResponseTest, a_request_without_a_server_is_lost) {
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(1);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    auto* service = create_service<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    rmw_service_info_t header{};
    bool taken = true;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    EXPECT_FALSE(taken);
}

TEST_F(RmwRequestResponseTest, a_request_that_fails_to_deserialize_is_dropped) {
    using Server = ::rmw::iox2::Server;

    const auto topic = create_test_topic();
    auto* service = create_service<BasicTypes>(topic);
    RMW_ASSERT_NE(service, nullptr);

    const auto* type_support = rosidl_typesupport_cpp::get_service_type_support_handle<BasicTypes>();
    const auto request_type_name = ::rmw::iox2::message_type_name(type_support->request_typesupport);
    const auto response_type_name = ::rmw::iox2::message_type_name(type_support->response_typesupport);
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto builder =
        native_node
            .service_builder(iox2::ServiceName::create(::rmw::iox2::names::service(topic.c_str()).c_str()).value())
            .request_response<Server::Payload, Server::Payload>()
            .request_user_header<Server::UserHeader>()
            .response_user_header<Server::UserHeader>();
    iox2::set_request_payload_type_details(builder,
                                           iox2::TypeDetail(iox2::TypeVariant::Dynamic,
                                                            request_type_name.c_str(),
                                                            ::rmw::iox2::SERIALIZED_PAYLOAD_ELEMENT_SIZE,
                                                            ::rmw::iox2::SERIALIZED_PAYLOAD_ALIGNMENT));
    iox2::set_response_payload_type_details(builder,
                                            iox2::TypeDetail(iox2::TypeVariant::Dynamic,
                                                             response_type_name.c_str(),
                                                             ::rmw::iox2::SERIALIZED_PAYLOAD_ELEMENT_SIZE,
                                                             ::rmw::iox2::SERIALIZED_PAYLOAD_ALIGNMENT));
    auto native_service = builder.resume_build()
                              .max_active_requests_per_client(::rmw::iox2::DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT)
                              .max_response_buffer_size(::rmw::iox2::MAX_RESPONSES_PER_REQUEST)
                              .request_payload_alignment(::rmw::iox2::SERIALIZED_PAYLOAD_ALIGNMENT)
                              .response_payload_alignment(::rmw::iox2::SERIALIZED_PAYLOAD_ALIGNMENT)
                              .open();
    ASSERT_TRUE(native_service.has_value()) << iox2::bb::into<const char*>(native_service.error());
    auto native_client = native_service->client_builder().initial_max_slice_len(4).create();
    ASSERT_TRUE(native_client.has_value());

    auto garbage = native_client->loan_slice_uninit(4);
    ASSERT_TRUE(garbage.has_value());
    std::fill_n(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(garbage->payload().data())), 4, 0xFF);
    auto pending_response = iox2::send(iox2::assume_init(std::move(garbage.value())));
    ASSERT_TRUE(pending_response.has_value());
    ASSERT_TRUE(pending_response->is_connected());

    rmw_service_info_t header{};
    BasicTypes::Request request;
    bool taken = true;
    EXPECT_EQ(rmw_take_request(service, &header, &request, &taken), RMW_RET_ERROR);
    rcutils_reset_error();
    EXPECT_FALSE(taken);
    EXPECT_FALSE(pending_response->is_connected());
}

TEST_F(RmwRequestResponseTest, a_response_reaches_the_client) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(42);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &request, &taken));
    ASSERT_TRUE(taken);

    auto response = make_response(43);
    ASSERT_RMW_OK(rmw_send_response(service, &request_header.request_id, &response));

    BasicTypes::Response received_response;
    rmw_service_info_t response_header{};
    ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_response, response);
    EXPECT_EQ(response_header.request_id.sequence_number, sequence_id);

    ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
    EXPECT_FALSE(taken);
}

TEST_F(RmwRequestResponseTest, a_request_and_its_response_are_sent_when_waking_up_fails) {
    auto topic = create_test_topic();
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service =
        native_node.service_builder(iox2::ServiceName::create(rmw::iox2::names::service(topic.c_str()).c_str()).value())
            .event()
            .deadline(iox2::bb::Duration::from_nanos(1))
            .create();
    ASSERT_TRUE(native_service.has_value());
    auto* service = create_service<BasicTypes>(topic);
    auto* client = create_client<BasicTypes>(topic);
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(42);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &request, &taken));
    ASSERT_TRUE(taken);

    auto response = make_response(43);
    ASSERT_RMW_OK(rmw_send_response(service, &request_header.request_id, &response));
    BasicTypes::Response received_response;
    rmw_service_info_t response_header{};
    ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_response, response);
}

TEST_F(RmwRequestResponseTest, request_and_response_rounds_beyond_the_active_request_limit_succeed) {
    using ::rmw::iox2::DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT;

    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    for (size_t round = 0; round < DEFAULT_MAX_ACTIVE_REQUESTS_PER_CLIENT + 1; ++round) {
        auto request = make_request(static_cast<int32_t>(round));
        int64_t sequence_id = 0;
        ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

        rmw_service_info_t request_header{};
        bool taken = false;
        ASSERT_RMW_OK(rmw_take_request(service, &request_header, &request, &taken));
        ASSERT_TRUE(taken);

        auto response = make_response(static_cast<int32_t>(round));
        ASSERT_RMW_OK(rmw_send_response(service, &request_header.request_id, &response));

        BasicTypes::Response received_response;
        rmw_service_info_t response_header{};
        ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
        ASSERT_TRUE(taken);
        EXPECT_EQ(received_response, response);
        EXPECT_EQ(response_header.request_id.sequence_number, sequence_id);
    }
}

TEST_F(RmwRequestResponseTest, a_response_waits_while_the_client_sends_more_requests) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(1);
    int64_t first_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &first_id));

    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &request, &taken));
    ASSERT_TRUE(taken);
    auto response = make_response(2);
    ASSERT_RMW_OK(rmw_send_response(service, &request_header.request_id, &response));

    int64_t second_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &second_id));

    BasicTypes::Response received_response;
    rmw_service_info_t response_header{};
    ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_response, response);
    EXPECT_EQ(response_header.request_id.sequence_number, first_id);
}

TEST_F(RmwRequestResponseTest, responses_find_their_requests) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    int64_t first_id = 0;
    int64_t second_id = 0;
    auto first_request = make_request(1);
    auto second_request = make_request(2);
    ASSERT_RMW_OK(rmw_send_request(client, &first_request, &first_id));
    ASSERT_RMW_OK(rmw_send_request(client, &second_request, &second_id));

    BasicTypes::Request request;
    rmw_service_info_t first_header{};
    rmw_service_info_t second_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &first_header, &request, &taken));
    ASSERT_TRUE(taken);
    ASSERT_RMW_OK(rmw_take_request(service, &second_header, &request, &taken));
    ASSERT_TRUE(taken);

    // Respond in reverse order: each response must still reach its own request.
    auto second_response = make_response(2);
    auto first_response = make_response(1);
    ASSERT_RMW_OK(rmw_send_response(service, &second_header.request_id, &second_response));
    ASSERT_RMW_OK(rmw_send_response(service, &first_header.request_id, &first_response));

    for (int i = 0; i < 2; ++i) {
        BasicTypes::Response response;
        rmw_service_info_t header{};
        ASSERT_RMW_OK(rmw_take_response(client, &header, &response, &taken));
        ASSERT_TRUE(taken);
        EXPECT_EQ(response.int32_value, header.request_id.sequence_number == first_id ? 1 : 2);
    }
}

TEST_F(RmwRequestResponseTest, each_client_gets_its_own_response) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* first_client = create_client<BasicTypes>(create_test_topic());
    auto* second_client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(first_client, nullptr);
    RMW_ASSERT_NE(second_client, nullptr);

    int64_t sequence_id = 0;
    auto first_request = make_request(1);
    auto second_request = make_request(2);
    ASSERT_RMW_OK(rmw_send_request(first_client, &first_request, &sequence_id));
    ASSERT_RMW_OK(rmw_send_request(second_client, &second_request, &sequence_id));

    for (int i = 0; i < 2; ++i) {
        BasicTypes::Request request;
        rmw_service_info_t header{};
        bool taken = false;
        ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
        ASSERT_TRUE(taken);
        auto response = make_response(request.int32_value);
        ASSERT_RMW_OK(rmw_send_response(service, &header.request_id, &response));
    }

    for (auto [client, expected] : {std::pair{first_client, 1}, std::pair{second_client, 2}}) {
        BasicTypes::Response response;
        rmw_service_info_t header{};
        bool taken = false;
        ASSERT_RMW_OK(rmw_take_response(client, &header, &response, &taken));
        ASSERT_TRUE(taken);
        EXPECT_EQ(response.int32_value, expected);
        ASSERT_RMW_OK(rmw_take_response(client, &header, &response, &taken));
        EXPECT_FALSE(taken);
    }
}

TEST_F(RmwRequestResponseTest, responding_to_a_client_that_is_gone_succeeds) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(1);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    rmw_service_info_t header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    ASSERT_TRUE(taken);

    destroy_client(client);

    auto response = make_response(1);
    EXPECT_RMW_OK(rmw_send_response(service, &header.request_id, &response));
}

TEST_F(RmwRequestResponseTest, responding_to_a_gone_client_after_taking_another_request_succeeds) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* gone_client = create_client<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(gone_client, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(1);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(gone_client, &request, &sequence_id));
    rmw_service_info_t gone_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &gone_header, &request, &taken));
    ASSERT_TRUE(taken);
    destroy_client(gone_client);

    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
    rmw_service_info_t header{};
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    ASSERT_TRUE(taken);

    auto response = make_response(1);
    EXPECT_RMW_OK(rmw_send_response(service, &gone_header.request_id, &response));
    EXPECT_RMW_OK(rmw_send_response(service, &header.request_id, &response));
}

TEST_F(RmwRequestResponseTest, responding_to_an_unknown_request_succeeds) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);

    rmw_request_id_t request_id{};
    request_id.sequence_number = 7;
    auto response = make_response(1);
    EXPECT_EQ(rmw_send_response(service, &request_id, &response), RMW_RET_OK);
}

TEST_F(RmwRequestResponseTest, a_server_is_ready_while_a_request_waits) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    EXPECT_FALSE(is_ready(service, rmw_time_t{0, 0}));

    auto request = make_request(1);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
    EXPECT_TRUE(is_ready(service, rmw_time_t{0, 0}));
    EXPECT_TRUE(is_ready(service, rmw_time_t{0, 0}));

    rmw_service_info_t header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    ASSERT_TRUE(taken);
    EXPECT_FALSE(is_ready(service, rmw_time_t{0, 0}));
}

TEST_F(RmwRequestResponseTest, a_client_is_ready_while_a_response_waits) {
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    auto request = make_request(1);
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));
    EXPECT_FALSE(is_ready(client, rmw_time_t{0, 0}));

    rmw_service_info_t header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
    ASSERT_TRUE(taken);
    auto response = make_response(1);
    ASSERT_RMW_OK(rmw_send_response(service, &header.request_id, &response));
    EXPECT_TRUE(is_ready(client, rmw_time_t{0, 0}));

    ASSERT_RMW_OK(rmw_take_response(client, &header, &response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_FALSE(is_ready(client, rmw_time_t{0, 0}));
}

TEST_F(RmwRequestResponseTest, requests_and_responses_wake_up_waiting_threads) {
    constexpr int32_t ROUND_TRIPS = 100;
    auto* service = create_service<BasicTypes>(create_test_topic());
    auto* client = create_client<BasicTypes>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    int32_t responded = 0;
    std::thread server([&] {
        while (responded < ROUND_TRIPS && is_ready(service, rmw_time_t{1, 0})) {
            BasicTypes::Request request;
            rmw_service_info_t header{};
            bool taken = false;
            EXPECT_RMW_OK(rmw_take_request(service, &header, &request, &taken));
            EXPECT_TRUE(taken);
            auto response = make_response(request.int32_value);
            EXPECT_RMW_OK(rmw_send_response(service, &header.request_id, &response));
            ++responded;
        }
    });
    int32_t completed = 0;
    int64_t sequence_id = 0;
    auto request = make_request(completed);
    while (completed < ROUND_TRIPS && rmw_send_request(client, &request, &sequence_id) == RMW_RET_OK
           && is_ready(client, rmw_time_t{1, 0})) {
        BasicTypes::Response response;
        rmw_service_info_t header{};
        bool taken = false;
        EXPECT_RMW_OK(rmw_take_response(client, &header, &response, &taken));
        EXPECT_TRUE(taken);
        EXPECT_EQ(response.int32_value, completed);
        request = make_request(++completed);
    }
    server.join();

    EXPECT_EQ(completed, ROUND_TRIPS);
    EXPECT_EQ(responded, ROUND_TRIPS);
}

TEST_F(RmwRequestResponseTest, a_self_contained_request_and_response_keep_their_values) {
    using rmw_iceoryx2_cxx_test_msgs::srv::SelfContained;

    auto* service = create_service<SelfContained>(create_test_topic());
    auto* client = create_client<SelfContained>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    SelfContained::Request request;
    request.basic_types_value.int32_value = 42;
    request.basic_types_value.float64_value = 1.5;
    request.last = 7;
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    SelfContained::Request received_request;
    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &received_request, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_request, request);

    SelfContained::Response response;
    response.basic_types_value.uint64_value = 43;
    ASSERT_RMW_OK(rmw_send_response(service, &request_header.request_id, &response));

    SelfContained::Response received_response;
    rmw_service_info_t response_header{};
    ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_response, response);
    EXPECT_EQ(response_header.request_id.sequence_number, sequence_id);
}

TEST_F(RmwRequestResponseTest, a_self_contained_request_gets_a_serialized_response) {
    using rmw_iceoryx2_cxx_test_msgs::srv::SelfContainedRequest;

    auto* service = create_service<SelfContainedRequest>(create_test_topic());
    auto* client = create_client<SelfContainedRequest>(create_test_topic());
    RMW_ASSERT_NE(service, nullptr);
    RMW_ASSERT_NE(client, nullptr);

    SelfContainedRequest::Request request;
    request.basic_types_value.int16_value = -3;
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &request, &sequence_id));

    SelfContainedRequest::Request received_request;
    rmw_service_info_t request_header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &request_header, &received_request, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_request, request);

    SelfContainedRequest::Response response;
    response.string_value = "response";
    ASSERT_RMW_OK(rmw_send_response(service, &request_header.request_id, &response));

    SelfContainedRequest::Response received_response;
    rmw_service_info_t response_header{};
    ASSERT_RMW_OK(rmw_take_response(client, &response_header, &received_response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_response, response);
}

TEST_F(RmwRequestResponseTest, an_iceoryx2_client_exchanges_self_contained_requests_and_responses_as_structs) {
    using rmw_iceoryx2_cxx_test_msgs::srv::SelfContained;

    const auto topic = create_test_topic();
    auto* service = create_service<SelfContained>(topic);
    RMW_ASSERT_NE(service, nullptr);
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service = open_native_self_contained_service(native_node, topic);
    ASSERT_TRUE(native_service.has_value()) << iox2::bb::into<const char*>(native_service.error());
    auto native_client = native_service->client_builder().create();
    ASSERT_TRUE(native_client.has_value());

    SelfContained::Request sent_request;
    sent_request.basic_types_value.int32_value = 42;
    sent_request.last = 7;
    auto request = native_client->loan_slice_uninit(::rmw::iox2::SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT);
    ASSERT_TRUE(request.has_value());
    std::memcpy(static_cast<void*>(request->payload_mut().data()), &sent_request, sizeof(sent_request));
    auto pending_response = iox2::send(iox2::assume_init(std::move(request.value())));
    ASSERT_TRUE(pending_response.has_value());

    SelfContained::Request received_request;
    rmw_service_info_t header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_request(service, &header, &received_request, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_request, sent_request);

    SelfContained::Response sent_response;
    sent_response.basic_types_value.uint64_value = 43;
    ASSERT_RMW_OK(rmw_send_response(service, &header.request_id, &sent_response));

    auto response = pending_response->receive();
    ASSERT_TRUE(response.has_value());
    ASSERT_TRUE(response->has_value());
    ASSERT_EQ(response->value().payload().number_of_bytes(), sizeof(SelfContained::Response));
    SelfContained::Response received_response;
    std::memcpy(static_cast<void*>(&received_response), response->value().payload().data(), sizeof(received_response));
    EXPECT_EQ(received_response, sent_response);
}

TEST_F(RmwRequestResponseTest, an_iceoryx2_server_exchanges_self_contained_requests_and_responses_as_structs) {
    using rmw_iceoryx2_cxx_test_msgs::srv::SelfContained;

    const auto topic = create_test_topic();
    auto* client = create_client<SelfContained>(topic);
    RMW_ASSERT_NE(client, nullptr);
    auto native_node = iox2::NodeBuilder().create<iox2::ServiceType::Ipc>().value();
    auto native_service = open_native_self_contained_service(native_node, topic);
    ASSERT_TRUE(native_service.has_value()) << iox2::bb::into<const char*>(native_service.error());
    auto native_server = native_service->server_builder().create();
    ASSERT_TRUE(native_server.has_value());

    SelfContained::Request sent_request;
    sent_request.basic_types_value.int32_value = 42;
    sent_request.last = 7;
    int64_t sequence_id = 0;
    ASSERT_RMW_OK(rmw_send_request(client, &sent_request, &sequence_id));

    auto request = native_server->receive();
    ASSERT_TRUE(request.has_value());
    ASSERT_TRUE(request->has_value());
    ASSERT_EQ(request->value().payload().number_of_bytes(), sizeof(SelfContained::Request));
    SelfContained::Request received_request;
    std::memcpy(static_cast<void*>(&received_request), request->value().payload().data(), sizeof(received_request));
    EXPECT_EQ(received_request, sent_request);

    SelfContained::Response sent_response;
    sent_response.basic_types_value.uint64_value = 43;
    auto response = request->value().loan_slice_uninit(::rmw::iox2::SELF_CONTAINED_PAYLOAD_ELEMENT_COUNT);
    ASSERT_TRUE(response.has_value());
    std::memcpy(static_cast<void*>(response->payload_mut().data()), &sent_response, sizeof(sent_response));
    ASSERT_TRUE(iox2::send(iox2::assume_init(std::move(response.value()))).has_value());

    SelfContained::Response received_response;
    rmw_service_info_t header{};
    bool taken = false;
    ASSERT_RMW_OK(rmw_take_response(client, &header, &received_response, &taken));
    ASSERT_TRUE(taken);
    EXPECT_EQ(received_response, sent_response);
}

} // namespace
