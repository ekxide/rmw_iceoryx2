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

class TransmissionDataListener : public rclcpp::Node {
public:
  explicit TransmissionDataListener(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("ros2_subscriber", options) {
    auto on_msg = [this](rmw_iceoryx2_interoperation_demo_msgs::msg::
                             TransmissionData::UniquePtr msg,
                         const rclcpp::MessageInfo &info) {
      const auto meta =
          "seq " + std::to_string(
                       info.get_rmw_message_info().publication_sequence_number);

      RCLCPP_INFO(get_logger(), "%s",
                  pretty::frame(pretty::Direction::Received, meta,
                                {{"x", std::to_string(msg->x)},
                                 {"y", std::to_string(msg->y)},
                                 {"funky", pretty::number(msg->funky)}})
                      .c_str());
    };
    m_subscription = create_subscription<
        rmw_iceoryx2_interoperation_demo_msgs::msg::TransmissionData>(
        "transmission_data", 10, on_msg);
  }

private:
  rclcpp::Subscription<
      rmw_iceoryx2_interoperation_demo_msgs::msg::TransmissionData>::SharedPtr
      m_subscription;
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
