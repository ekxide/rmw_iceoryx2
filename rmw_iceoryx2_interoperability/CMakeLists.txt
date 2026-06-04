cmake_minimum_required(VERSION 3.8)
project(rmw_iceoryx2_interoperability)

find_package(ament_cmake REQUIRED)
find_program(CARGO cargo REQUIRED)

set(generated_include_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(generated_header ${generated_include_dir}/${PROJECT_NAME}/${PROJECT_NAME}.h)

# Build the crate so its build.rs runs cbindgen and emits the C++ header.
add_custom_command(
  OUTPUT ${generated_header}
  COMMAND ${CMAKE_COMMAND} -E env RMW_IOX2_INTEROP_HEADER_OUT=${generated_header}
          ${CARGO} build --release
          --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
          --target-dir ${CMAKE_CURRENT_BINARY_DIR}/cargo
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
    ${CMAKE_CURRENT_SOURCE_DIR}/build.rs
    ${CMAKE_CURRENT_SOURCE_DIR}/cbindgen.toml
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lib.rs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/constants.rs
  COMMENT "Generating ${PROJECT_NAME} C++ header via cbindgen"
  VERBATIM
)

add_custom_target(${PROJECT_NAME}_header ALL DEPENDS ${generated_header})

install(DIRECTORY ${generated_include_dir}/ DESTINATION include)

ament_export_include_directories(include)
ament_package()
