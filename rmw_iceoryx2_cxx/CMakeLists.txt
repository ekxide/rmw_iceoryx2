cmake_minimum_required(VERSION 3.5)
project(rmw_iceoryx2_cxx)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # NOTE: Remove -Wno-unused-parameter after implementation
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(CODE_COVERAGE AND
  CMAKE_SYSTEM_NAME MATCHES "Linux" AND
  CMAKE_C_COMPILER_ID STREQUAL "GNU")
  message("GCov code coverage enabled")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_EXE_LINKER_FLAGS "--coverage")
endif()

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(iceoryx2-cxx REQUIRED)

set(AMENT_PACKAGES
    rcutils
    rcpputils
    rmw
    rosidl_runtime_cpp
    rosidl_typesupport_introspection_cpp
    rosidl_typesupport_fastrtps_cpp
    fastrtps
)
foreach(package ${AMENT_PACKAGES})
    find_package(${package} REQUIRED)
endforeach()

# ----------------------------------------------------------------------------
# Library
# ----------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/iox2/context_impl.cpp
  src/iox2/guard_condition_impl.cpp
  src/iox2/names.cpp
  src/iox2/node_impl.cpp
  src/iox2/publisher_impl.cpp
  src/iox2/subscriber_impl.cpp
  src/iox2/waitset_impl.cpp
  src/message/introspect.cpp
  src/message/serialize.cpp
  src/rmw/client.cpp
  src/rmw/event.cpp
  src/rmw/feature.cpp
  src/rmw/gid.cpp
  src/rmw/graph.cpp
  src/rmw/guard_condition.cpp
  src/rmw/identifier.cpp
  src/rmw/init.cpp
  src/rmw/logging.cpp
  src/rmw/node.cpp
  src/rmw/publisher.cpp
  src/rmw/qos.cpp
  src/rmw/serialize.cpp
  src/rmw/service.cpp
  src/rmw/subscription.cpp
  src/rmw/waitset.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Manual linking required as iceoryx2 is not an ament package
target_link_libraries(${PROJECT_NAME}
  iceoryx2-cxx::static-lib-cxx
  rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
)

# ----------------------------------------------------------------------------
# Install
# ----------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION share/${PROJECT_NAME}/cmake
  NAMESPACE ${PROJECT_NAME}::
)

# ----------------------------------------------------------------------------
# RMW Registration
# ----------------------------------------------------------------------------

configure_rmw_library(${PROJECT_NAME})
register_rmw_implementation(
  "cpp:rosidl_typesupport_fastrtps_cpp:rosidl_typesupport_introspection_cpp"

)

# ----------------------------------------------------------------------------
# Package
# ----------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_rmw_iceoryx2_cxx 
    test/testing/base.cpp
    test/test_iox2_context_impl.cpp
    test/test_iox2_guard_condition_impl.cpp
    test/test_iox2_message_introspection.cpp
    test/test_iox2_node_impl.cpp
    test/test_iox2_publisher_impl.cpp
    test/test_iox2_sample_registry.cpp
    test/test_iox2_subscriber_impl.cpp
    test/test_iox2_waitset_impl.cpp
    test/test_rmw_allocator.cpp
    test/test_rmw_gid.cpp
    test/test_rmw_graph.cpp
    test/test_rmw_guard_condition.cpp
    test/test_rmw_init.cpp
    test/test_rmw_logging.cpp
    test/test_rmw_node.cpp
    test/test_rmw_publisher.cpp
    test/test_rmw_serialize.cpp
    test/test_rmw_subscription.cpp
    test/test_rmw_publish_subscribe.cpp
    test/test_rmw_waitset.cpp
  )
  target_link_libraries(test_rmw_iceoryx2_cxx 
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_rmw_iceoryx2_cxx
    rmw_iceoryx2_cxx_test_msgs
    rosidl_typesupport_cpp
  )

endif()

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGES}
)
ament_export_targets(${PROJECT_NAME}Targets)

ament_package()
