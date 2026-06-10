// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/basic_types.hpp"
#include "rmw_iceoryx2_talker_demo_nodes/pretty.hpp"

class BasicTypesListenerNode : public rclcpp::Node {
public:
  BasicTypesListenerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("listener_basic_types", options) {

    auto on_msg =
        [this](rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes::UniquePtr msg,
               const rclcpp::MessageInfo &info) -> void {
      const auto meta =
          "seq " + std::to_string(
                       info.get_rmw_message_info().publication_sequence_number);

      RCLCPP_INFO(this->get_logger(), "%s",
                  pretty::frame(
                      pretty::Direction::Received, meta,
                      {{"bool_value", msg->bool_value ? "true" : "false"},
                       {"byte_value",
                        std::to_string(static_cast<uint32_t>(msg->byte_value))},
                       {"char_value",
                        std::to_string(static_cast<uint32_t>(msg->char_value))},
                       {"float32_value", pretty::number(msg->float32_value)},
                       {"float64_value", pretty::number(msg->float64_value)},
                       {"int8_value",
                        std::to_string(static_cast<int>(msg->int8_value))},
                       {"uint8_value", std::to_string(static_cast<uint32_t>(
                                           msg->uint8_value))},
                       {"int16_value", std::to_string(msg->int16_value)},
                       {"uint16_value", std::to_string(msg->uint16_value)},
                       {"int32_value", std::to_string(msg->int32_value)},
                       {"uint32_value", std::to_string(msg->uint32_value)},
                       {"int64_value", std::to_string(msg->int64_value)},
                       {"uint64_value", std::to_string(msg->uint64_value)}})
                      .c_str());
    };
    subscription_ =
        this->create_subscription<rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes>(
            "basic_types", 10, on_msg);
  }

private:
  rclcpp::Subscription<rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes>::SharedPtr
      subscription_;
};

int main(int argc, char *argv[]) {
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);

  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_logger_service(false);
  options.enable_rosout(false);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicTypesListenerNode>(options));
  rclcpp::shutdown();
  return 0;
}
