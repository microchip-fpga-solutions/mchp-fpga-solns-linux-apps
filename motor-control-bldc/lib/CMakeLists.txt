cmake_minimum_required(VERSION 3.14)
project(motor_lib_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable Position Independent Code for RISC-V shared library compatibility
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable warnings for better code quality
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(pybind11 REQUIRED)

# Static library with PIC (needed for linking into shared Python module)
add_library(motor_lib STATIC motor.cpp)
set_target_properties(motor_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(motor_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link stdc++fs for GCC < 9 (std::filesystem support)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(motor_lib PUBLIC stdc++fs)
endif()

# Python bindings
pybind11_add_module(motor_lib_py bindings.cpp)
target_link_libraries(motor_lib_py PRIVATE motor_lib)
target_include_directories(motor_lib_py PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
