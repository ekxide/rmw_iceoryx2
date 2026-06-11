# Copyright (c) 2026 by Ekxide IO GmbH All rights reserved.
#
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

cmake_minimum_required(VERSION 3.8)
project(rmw_iceoryx2_interoperation_demo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TransmissionData.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
