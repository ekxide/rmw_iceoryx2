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

using namespace std::chrono_literals;

class StringsTalkerNode : public rclcpp::Node {
public:
  StringsTalkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("talker_strings", options), m_count(0) {

    m_publisher =
        this->create_publisher<rmw_iceoryx2_cxx_test_msgs::msg::Strings>(
            "strings", 10);

    auto publish = [this]() -> void {
      auto loan = m_publisher->borrow_loaned_message();
      loan.get().string_value = "Hello " + std::to_string(m_count);
      m_publisher->publish(std::move(loan));
      m_count++;
    };

    m_timer = this->create_wall_timer(2s, publish);
  }

private:
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<rmw_iceoryx2_cxx_test_msgs::msg::Strings>::SharedPtr
      m_publisher;
  size_t m_count{0};
};

int main(int argc, char *argv[]) {
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);

  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_logger_service(false);
  options.enable_rosout(false);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringsTalkerNode>(options));
  rclcpp::shutdown();
  return 0;
}
