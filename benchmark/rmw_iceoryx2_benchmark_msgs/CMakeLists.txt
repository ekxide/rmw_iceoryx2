cmake_minimum_required(VERSION 3.8)
project(rmw_iceoryx2_benchmark_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sample.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
