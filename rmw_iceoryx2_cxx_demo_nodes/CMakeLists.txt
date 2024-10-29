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

add_executable(talker src/publisher_lambda_function.cpp)
ament_target_dependencies(talker rclcpp rmw_iceoryx2_cxx_test_msgs)

add_executable(listener src/subscriber_lambda_function.cpp)
ament_target_dependencies(listener rclcpp rmw_iceoryx2_cxx_test_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
