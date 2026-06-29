# Host-side unit tests for the pure PID core (PidCore.h). No ESP-IDF required.
#
#   cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(pidcore_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(test_pidcore test_pidcore.cpp)
target_include_directories(test_pidcore PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
if(NOT MSVC)
    target_compile_options(test_pidcore PRIVATE -Wall -Wextra)
endif()

add_test(NAME pidcore COMMAND test_pidcore)
