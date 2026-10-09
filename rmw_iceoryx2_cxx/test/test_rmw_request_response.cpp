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
#include "rosidl_typesupport_cpp/service_type_support.hpp"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <algorithm>

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

    static BasicTypes::Request make_request(int32_t value) {
        BasicTypes::Request request;
        request.int32_value = value;
        request.string_value = "request " + std::to_string(value);
        return request;
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

} // namespace
