# Copyright (c) 2026 by Mykhaylo Marfeychuk All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

cmake_minimum_required(VERSION 3.22)
project(rmw_iceoryx2_api_benchmarks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_iceoryx2_benchmark_msgs REQUIRED)
find_package(rmw_iceoryx2_benchmark_nodes REQUIRED)

foreach(benchmark idle_spin guard_condition graph_change graph_query node endpoint service)
  add_executable(${benchmark} src/${benchmark}.cpp)
  target_include_directories(${benchmark} PRIVATE ${rmw_iceoryx2_benchmark_nodes_INCLUDE_DIRS})
  target_link_libraries(${benchmark} PRIVATE
    rclcpp::rclcpp
    rmw_iceoryx2_benchmark_msgs::rmw_iceoryx2_benchmark_msgs__rosidl_typesupport_cpp)
  install(TARGETS ${benchmark} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
