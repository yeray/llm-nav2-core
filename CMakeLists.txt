cmake_minimum_required(VERSION 3.16)
project(llm_nav2_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -Wpedantic)

# Pure Hexagonal Core Library (Zero external dependencies: no ROS 2, no llama.h)
add_library(llm_nav2_domain STATIC
  src/domain/waypoint_registry.cpp
  src/domain/gbnf_builder.cpp
  src/domain/nav_command_processor.cpp
)

target_include_directories(llm_nav2_domain PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Unit Testing
enable_testing()

add_executable(test_domain_core
  tests/test_domain_core.cpp
)
target_link_libraries(test_domain_core PRIVATE llm_nav2_domain)

add_test(NAME test_domain_core COMMAND test_domain_core)
