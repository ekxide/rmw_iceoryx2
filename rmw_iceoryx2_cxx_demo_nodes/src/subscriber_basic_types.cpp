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

class StringsListenerNode : public rclcpp::Node {
public:
  StringsListenerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("listener_basic_types", options) {

    auto on_msg =
        [this](rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes::UniquePtr msg)
        -> void {
      RCLCPP_DEBUG(this->get_logger(), "Got message");
      RCLCPP_DEBUG(this->get_logger(),
                   "Message content:\n"
                   "  bool_value: %d\n"
                   "  byte_value: %u\n"
                   "  char_value: %u\n"
                   "  float32_value: %.2f\n"
                   "  float64_value: %.2f\n"
                   "  int8_value: %d\n"
                   "  uint8_value: %u\n"
                   "  int16_value: %d\n"
                   "  uint16_value: %u\n"
                   "  int32_value: %d\n"
                   "  uint32_value: %u\n"
                   "  int64_value: %ld\n"
                   "  uint64_value: %lu",
                   msg->bool_value, static_cast<uint32_t>(msg->byte_value),
                   static_cast<uint32_t>(msg->char_value), msg->float32_value,
                   msg->float64_value, msg->int8_value, msg->uint8_value,
                   msg->int16_value, msg->uint16_value, msg->int32_value,
                   msg->uint32_value, msg->int64_value, msg->uint64_value);
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
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_logger_service(false);
  options.enable_rosout(false);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringsListenerNode>(options));
  rclcpp::shutdown();
  return 0;
}
