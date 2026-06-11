// Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/strings.hpp"
#include "rmw_iceoryx2_talker_demo_nodes/pretty.hpp"

class StringsListenerNode : public rclcpp::Node {
public:
  StringsListenerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("listener_strings", options) {

    auto on_msg =
        [this](rmw_iceoryx2_cxx_test_msgs::msg::Strings::UniquePtr msg,
               const rclcpp::MessageInfo &info) -> void {
      const auto meta =
          "seq " + std::to_string(
                       info.get_rmw_message_info().publication_sequence_number);

      RCLCPP_INFO(this->get_logger(), "%s",
                  pretty::frame(pretty::Direction::Received, meta,
                                {{"string_value", msg->string_value}})
                      .c_str());
    };
    subscription_ =
        this->create_subscription<rmw_iceoryx2_cxx_test_msgs::msg::Strings>(
            "strings", 10, on_msg);
  }

private:
  rclcpp::Subscription<rmw_iceoryx2_cxx_test_msgs::msg::Strings>::SharedPtr
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
  rclcpp::spin(std::make_shared<StringsListenerNode>(options));
  rclcpp::shutdown();
  return 0;
}
