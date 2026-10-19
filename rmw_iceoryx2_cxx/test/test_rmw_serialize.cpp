// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include <gtest/gtest.h>

#include "fastcdr/Cdr.h"
#include "rmw/rmw.h"
#include "rmw_iceoryx2_cxx/impl/message/introspection.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/defaults.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.h"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "testing/assertions.hpp"
#include "testing/base.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace
{

using namespace rmw::iox2::testing;

class RmwSerializeTest : public TestBase
{
protected:
    void SetUp() override {
    }

    void TearDown() override {
        print_rmw_errors();
    }
};

TEST_F(RmwSerializeTest, serialize_deserialize_pod_type) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Defaults;

    Defaults input{};
    input.int64_value = 777;

    rmw_serialized_message_t serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Defaults), &test_allocator()));

    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Defaults>(), &serialized_msg));
    ASSERT_NE(serialized_msg.buffer, nullptr);
    ASSERT_NE(serialized_msg.buffer_length, 0);
    ASSERT_NE(serialized_msg.buffer_capacity, 0);

    Defaults output{};
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Defaults>(), &output));
    ASSERT_EQ(input, output);
}

TEST_F(RmwSerializeTest, serialize_deserialize_non_pod_type) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    Strings input{};
    input.string_value = "GloryToHypnoToad";

    rmw_serialized_message_t serialized_msg{};
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, sizeof(Strings), &test_allocator()));

    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &serialized_msg));
    ASSERT_NE(serialized_msg.buffer, nullptr);
    ASSERT_NE(serialized_msg.buffer_length, 0);
    ASSERT_NE(serialized_msg.buffer_capacity, 0);

    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Strings>(), &output));
    ASSERT_EQ(input, output);
}

TEST_F(RmwSerializeTest, serialize_deserialize_c_type_support) {
    const auto* type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(rmw_iceoryx2_cxx_test_msgs, msg, Strings);

    rmw_iceoryx2_cxx_test_msgs__msg__Strings input;
    ASSERT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__Strings__init(&input));
    ASSERT_TRUE(rosidl_runtime_c__String__assign(&input.string_value, "GloryToHypnoToad"));

    rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, 0, &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, type_support, &serialized_msg));

    rmw_iceoryx2_cxx_test_msgs__msg__Strings output;
    ASSERT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__Strings__init(&output));
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, type_support, &output));
    EXPECT_TRUE(rmw_iceoryx2_cxx_test_msgs__msg__Strings__are_equal(&input, &output));

    rmw_iceoryx2_cxx_test_msgs__msg__Strings__fini(&output);
    rmw_iceoryx2_cxx_test_msgs__msg__Strings__fini(&input);
    ASSERT_RMW_OK(rmw_serialized_message_fini(&serialized_msg));
}

TEST_F(RmwSerializeTest, serialize_resizes_the_serialized_message) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    Strings input{};
    input.string_value = "GloryToHypnoToad";

    rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, 0, &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &serialized_msg));

    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Strings>(), &output));
    ASSERT_EQ(input, output);
    ASSERT_RMW_OK(rmw_serialized_message_fini(&serialized_msg));
}

TEST_F(RmwSerializeTest, deserialize_truncated_message_fails) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;

    rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, 2, &test_allocator()));
    serialized_msg.buffer_length = 2;

    Strings output{};
    EXPECT_RMW_ERR(RMW_RET_ERROR, rmw_deserialize(&serialized_msg, test_type_support<Strings>(), &output));
    ASSERT_RMW_OK(rmw_serialized_message_fini(&serialized_msg));
}

TEST_F(RmwSerializeTest, the_serialization_format_is_cdr) {
    EXPECT_STREQ(rmw_get_serialization_format(), "cdr");
}

TEST_F(RmwSerializeTest, serialized_messages_start_with_the_cdr_encapsulation_header) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;
    constexpr std::array<uint8_t, ::rmw::iox2::CDR_ENCAPSULATION_SIZE> LITTLE_ENDIAN_CDR_HEADER{0x00, 0x01, 0x00, 0x00};

    Strings input{};
    input.string_value = "GloryToHypnoToad";

    rmw_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
    ASSERT_RMW_OK(rmw_serialized_message_init(&serialized_msg, 0, &test_allocator()));
    ASSERT_RMW_OK(rmw_serialize(&input, test_type_support<Strings>(), &serialized_msg));

    ASSERT_GE(serialized_msg.buffer_length, LITTLE_ENDIAN_CDR_HEADER.size());
    EXPECT_TRUE(std::equal(LITTLE_ENDIAN_CDR_HEADER.begin(), LITTLE_ENDIAN_CDR_HEADER.end(), serialized_msg.buffer));
    ASSERT_RMW_OK(rmw_serialized_message_fini(&serialized_msg));
}

TEST_F(RmwSerializeTest, a_message_serialized_in_big_endian_deserializes) {
    using rmw_iceoryx2_cxx_test_msgs::msg::Strings;
    const auto* handle = get_message_typesupport_handle(test_type_support<Strings>(),
                                                        rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
    ASSERT_NE(handle, nullptr);
    const auto* callbacks = static_cast<const message_type_support_callbacks_t*>(handle->data);

    Strings input{};
    input.string_value = "GloryToHypnoToad";

    std::vector<uint8_t> buffer(::rmw::iox2::CDR_ENCAPSULATION_SIZE + callbacks->get_serialized_size(&input));
    eprosima::fastcdr::FastBuffer fast_buffer(reinterpret_cast<char*>(buffer.data()), buffer.size());
    eprosima::fastcdr::Cdr serializer(
        fast_buffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS, eprosima::fastcdr::CdrVersion::DDS_CDR);
    serializer.serialize_encapsulation();
    ASSERT_TRUE(callbacks->cdr_serialize(&input, serializer));

    rmw_serialized_message_t serialized_msg{
        buffer.data(), serializer.get_serialized_data_length(), buffer.size(), test_allocator()};
    Strings output{};
    ASSERT_RMW_OK(rmw_deserialize(&serialized_msg, test_type_support<Strings>(), &output));
    EXPECT_EQ(input, output);
}

} // namespace
