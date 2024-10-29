// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "rmw_iceoryx2_cxx_test_msgs/msg/basic_types.hpp"

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node {
public:
  TalkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("talker", options), m_count(0) {

    m_publisher =
        this->create_publisher<rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes>(
            "basic_types", 10);

    auto publish = [this]() -> void {
      auto loan = m_publisher->borrow_loaned_message();

      auto &msg = loan.get();
      msg.bool_value = static_cast<bool>(m_count % 2);
      msg.byte_value = static_cast<uint8_t>(m_count);
      msg.char_value = static_cast<char>(m_count % 128);
      msg.float32_value = static_cast<float>(m_count);
      msg.float64_value = static_cast<double>(m_count);
      msg.int8_value = static_cast<int8_t>(m_count);
      msg.uint8_value = static_cast<uint8_t>(m_count);
      msg.int16_value = static_cast<int16_t>(m_count);
      msg.uint16_value = static_cast<uint16_t>(m_count);
      msg.int32_value = static_cast<int32_t>(m_count);
      msg.uint32_value = static_cast<uint32_t>(m_count);
      msg.int64_value = static_cast<int64_t>(m_count);
      msg.uint64_value = static_cast<uint64_t>(m_count);

      RCLCPP_DEBUG(this->get_logger(), "Publishing message");
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
                   msg.bool_value, static_cast<uint32_t>(msg.byte_value),
                   static_cast<uint32_t>(msg.char_value), msg.float32_value,
                   msg.float64_value, msg.int8_value, msg.uint8_value,
                   msg.int16_value, msg.uint16_value, msg.int32_value,
                   msg.uint32_value, msg.int64_value, msg.uint64_value);

      m_publisher->publish(std::move(loan));
      m_count++;
    };

    m_timer = this->create_wall_timer(2s, publish);
  }

private:
  rclcpp::TimerBase::SharedPtr m_timer;
  rclcpp::Publisher<rmw_iceoryx2_cxx_test_msgs::msg::BasicTypes>::SharedPtr
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
  rclcpp::spin(std::make_shared<TalkerNode>(options));
  rclcpp::shutdown();
  return 0;
}
