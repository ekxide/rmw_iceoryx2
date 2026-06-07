# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

cmake_minimum_required(VERSION 3.8)
project(rmw_iceoryx2_interoperation_demo_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_iceoryx2_interoperation_demo_msgs REQUIRED)

add_executable(ros2_publisher src/ros2_publisher.cpp)
target_link_libraries(ros2_publisher PRIVATE
  rclcpp::rclcpp
  rmw_iceoryx2_interoperation_demo_msgs::rmw_iceoryx2_interoperation_demo_msgs__rosidl_typesupport_cpp)

add_executable(ros2_subscriber src/ros2_subscriber.cpp)
target_link_libraries(ros2_subscriber PRIVATE
  rclcpp::rclcpp
  rmw_iceoryx2_interoperation_demo_msgs::rmw_iceoryx2_interoperation_demo_msgs__rosidl_typesupport_cpp)

install(TARGETS
  ros2_publisher
  ros2_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
