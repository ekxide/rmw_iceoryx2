# Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
# 
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: Apache-2.0 OR MIT

cmake_minimum_required(VERSION 3.8)
project(rmw_iceoryx2_cxx_demo_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rmw_iceoryx2_cxx_test_msgs REQUIRED)

add_executable(talker_basic_types src/publisher_basic_types.cpp)
ament_target_dependencies(talker_basic_types rclcpp rmw_iceoryx2_cxx_test_msgs)

add_executable(listener_basic_types src/subscriber_basic_types.cpp)
ament_target_dependencies(listener_basic_types rclcpp rmw_iceoryx2_cxx_test_msgs)

add_executable(talker_strings src/publisher_strings.cpp)
ament_target_dependencies(talker_strings rclcpp rmw_iceoryx2_cxx_test_msgs)

add_executable(listener_strings src/subscriber_strings.cpp)
ament_target_dependencies(listener_strings rclcpp rmw_iceoryx2_cxx_test_msgs)

install(TARGETS
  talker_basic_types
  listener_basic_types
  talker_strings
  listener_strings
  DESTINATION lib/${PROJECT_NAME})

ament_package()
