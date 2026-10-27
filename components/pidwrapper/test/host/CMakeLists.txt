# Host-side unit tests for the pure PID core (PidCore.h). No ESP-IDF required.
#
#   cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(pidcore_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(test_pidcore test_pidcore.cpp)
add_executable(test_pid_behavior test_pid_behavior.cpp)
foreach(t test_pidcore test_pid_behavior)
    target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
    if(NOT MSVC)
        target_compile_options(${t} PRIVATE -Wall -Wextra)
    endif()
endforeach()

add_test(NAME pidcore COMMAND test_pidcore)
add_test(NAME pid_behavior COMMAND test_pid_behavior)
