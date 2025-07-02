# =====================================
# Draco4 EtherCAT Motor Controller
# =====================================
# 
# CMake build configuration for Synapticon servo motor control system
# using EtherCAT communication via SOEM library.
#
# Requirements:
#   - CMake 3.16+
#   - SOEM library (system-installed)
#   - C++17 compiler support

cmake_minimum_required(VERSION 3.16)
project(draco4_ethercat 
    VERSION 1.0.0
    DESCRIPTION "Synapticon EtherCAT Motor Controller"
    LANGUAGES CXX C
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SOEM (Simple Open EtherCAT Master) library
find_package(PkgConfig REQUIRED)
pkg_check_modules(SOEM REQUIRED soem)

# Include directories
include_directories(include)
include_directories(${SOEM_INCLUDE_DIRS})

# =====================================
# Core Library
# =====================================

add_library(draco4_ethercat
    src/motor_controller.cpp
    src/motor_configuration.cpp
    src/motor_sdo_manager.cpp
    src/motor_constants.cpp
    src/network_controller.cpp
    src/motor_manager.cpp
    src/robot_controller.cpp
)

target_link_libraries(draco4_ethercat 
    ${SOEM_LIBRARIES}
    pthread
)

# =====================================
# Legacy Reference Implementation
# =====================================

add_executable(draco_test_legacy legacy/draco_test.c)
target_include_directories(draco_test_legacy PRIVATE ${SOEM_INCLUDE_DIRS})
target_link_libraries(draco_test_legacy ${SOEM_LIBRARIES} pthread)

# =====================================
# Test Suite
# =====================================

# Core functionality tests

# Moved to legacy: test_soem_multi_instance (SOEM validation test)

# NetworkController validation test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_network_controller.cpp)
    add_executable(test_network_controller tests/test_network_controller.cpp)
    target_link_libraries(test_network_controller draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_position.cpp)
    add_executable(test_jd8_position tests/test_jd8_position.cpp)
    target_link_libraries(test_jd8_position draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_torque.cpp)
    add_executable(test_jd8_torque tests/test_jd8_torque.cpp)
    target_link_libraries(test_jd8_torque draco4_ethercat)
endif()

# Moved to legacy: test_motor_detection (development utility)

# Multi-motor integration tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_multi_motor_simulation.cpp)
    add_executable(test_multi_motor_simulation tests/test_multi_motor_simulation.cpp)
    target_link_libraries(test_multi_motor_simulation draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_multi_motor_hardware.cpp)
    add_executable(test_multi_motor_hardware tests/test_multi_motor_hardware.cpp)
    target_link_libraries(test_multi_motor_hardware draco4_ethercat)
endif()

# Phase 2: Multi-network integration tests
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_multi_network_simulation.cpp)
    add_executable(test_multi_network_simulation tests/test_multi_network_simulation.cpp)
    target_link_libraries(test_multi_network_simulation draco4_ethercat)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_multi_network_hardware.cpp)
    add_executable(test_multi_network_hardware tests/test_multi_network_hardware.cpp)
    target_link_libraries(test_multi_network_hardware draco4_ethercat)
endif()


# Moved to legacy: test_sdo_integration (development utility)



# JD8 velocity test
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_jd8_velocity.cpp)
    add_executable(test_jd8_velocity tests/test_jd8_velocity.cpp)
    target_link_libraries(test_jd8_velocity draco4_ethercat)
endif()


