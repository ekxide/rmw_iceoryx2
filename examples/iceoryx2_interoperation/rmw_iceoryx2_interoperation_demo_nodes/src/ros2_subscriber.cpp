// Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
//
// This program and the accompanying materials are made available under the
// terms of the Apache Software License 2.0 which is available at
// https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
// which is available at https://opensource.org/licenses/MIT.
//
// SPDX-License-Identifier: Apache-2.0 OR MIT

#include "rclcpp/rclcpp.hpp"
#include "iceoryx2_interoperation_demo_msgs/msg/transmission_data.hpp"

class TransmissionDataListener : public rclcpp::Node {
public:
  explicit TransmissionDataListener(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("ros2_subscriber", options) {
    auto on_msg = [this](iceoryx2_interoperation_demo_msgs::msg::TransmissionData::UniquePtr msg) {
      RCLCPP_INFO(get_logger(), "received: TransmissionData { x: %d, y: %d, funky: %.2f }",
                  msg->x, msg->y, msg->funky);
    };
    m_subscription = create_subscription<iceoryx2_interoperation_demo_msgs::msg::TransmissionData>(
        "transmission_data", 10, on_msg);
  }

private:
  rclcpp::Subscription<iceoryx2_interoperation_demo_msgs::msg::TransmissionData>::SharedPtr m_subscription;
};

int main(int argc, char *argv[]) {
  rclcpp::NodeOptions options;
  options.start_parameter_services(false);
  options.start_parameter_event_publisher(false);
  options.enable_logger_service(false);
  options.enable_rosout(false);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransmissionDataListener>(options));
  rclcpp::shutdown();
  return 0;
}
