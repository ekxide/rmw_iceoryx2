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
    rosidl_runtime_c
    rosidl_runtime_cpp
    rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp
)
foreach(package ${AMENT_PACKAGES})
    find_package(${package} REQUIRED)
endforeach()

# ----------------------------------------------------------------------------
# Library
# ----------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/rmw_client.cpp
  src/rmw_event.cpp
  src/rmw_graph.cpp
  src/rmw_guard_condition.cpp
  src/rmw_identifier.cpp
  src/rmw_init.cpp
  src/rmw_logging.cpp
  src/rmw_node.cpp
  src/rmw_node_impl.cpp
  src/rmw_publisher.cpp
  src/rmw_qos.cpp
  src/rmw_serialize.cpp
  src/rmw_service.cpp
  src/rmw_subscription.cpp
  src/rmw_waitset.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Manual linking required as iceoryx2 is not an ament package
target_link_libraries(${PROJECT_NAME}
  iceoryx2-cxx::static-lib-cxx
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
  "c:rosidl_typesupport_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_cpp:rosidl_typesupport_introspection_cpp"
)

# ----------------------------------------------------------------------------
# Package
# ----------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rmw_iceoryx2_cxx 
    test/test_rmw_allocator_helpers.cpp
    test/test_rmw_graph.cpp
    test/test_rmw_init.cpp
    test/test_rmw_logging.cpp
    test/test_rmw_node.cpp
    test/test_rmw_node_impl.cpp
  )
  target_link_libraries(test_rmw_iceoryx2_cxx ${PROJECT_NAME})
endif()

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_PACKAGES}
)
ament_export_targets(${PROJECT_NAME}Targets)

ament_package()
