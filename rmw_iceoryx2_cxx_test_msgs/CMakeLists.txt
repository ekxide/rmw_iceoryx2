# Copyright (c) 2024 by Ekxide IO GmbH All rights reserved.
# 
# This program and the accompanying materials are made available under the
# terms of the Apache Software License 2.0 which is available at
# https://www.apache.org/licenses/LICENSE-2.0, or the MIT license
# which is available at https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: Apache-2.0 OR MIT

cmake_minimum_required(VERSION 3.5)
project(rmw_iceoryx2_cxx_test_msgs)

set(AMENT_PACKAGES
  ament_cmake
  rosidl_default_generators
  test_interface_files
)
foreach(package ${AMENT_PACKAGES})
    find_package(${package} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${test_interface_files_MSG_FILES}
  DEPENDENCIES test_interface_files
)

ament_export_include_directories("${CMAKE_INSTALL_PREFIX}/include")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
