// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_interoperation_demo_msgs/msg/transmission_data.hpp"
#include "rmw_iceoryx2_interoperation_demo_nodes/pretty.hpp"

using namespace std::chrono_literals;

class TransmissionDataTalker : public rclcpp::Node {
public:
  explicit TransmissionDataTalker(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("ros2_publisher", options) {
    m_publisher = create_publisher<
        rmw_iceoryx2_interoperation_demo_msgs::msg::TransmissionData>(
        "transmission_data", 10);

    auto publish = [this]() {
      m_count++;
      auto loan = m_publisher->borrow_loaned_message();
      auto &msg = loan.get();
      msg.x = m_count;
      msg.y = m_count * 3;
      msg.funky = static_cast<double>(m_count) * 812.12;

      RCLCPP_INFO(get_logger(), "%s",
                  pretty::frame(pretty::Direction::Sent, "",
                                {{"x", std::to_string(msg.x)},
                                 {"y", std::to_string(msg.y)},
                                 {"funky", pretty::number(msg.funky)}})
                      .c_str());
      m_publisher->publish(std::move(loan));
    };
    m_timer = create_wall_timer(1s, publish);
  }

private:
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<
      rmw_iceoryx2_interoperation_demo_msgs::msg::TransmissionData>::SharedPtr
      m_publisher;
  int32_t m_count{0};
};

int main(int argc, char *argv[]) {
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_logger_service(false);
  options.enable_rosout(false);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransmissionDataTalker>(options));
  rclcpp::shutdown();
  return 0;
}
