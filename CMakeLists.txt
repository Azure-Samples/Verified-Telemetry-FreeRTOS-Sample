# Copyright (c) Microsoft Corporation. All rights reserved.
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)
if(NOT VENDOR MATCHES "^PC$")
    set(CMAKE_TOOLCHAIN_FILE cmake/${VENDOR}_gcc.cmake)
endif()

project(iot-middleware-sample C ASM)
set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# include common utilities
include(common/utilities)

# Set FreeRTOS path
if(NOT FREERTOS_PATH)
    message(STATUS "No FREERTOS_PATH specified: Fetching FreeRTOS to path <${FREERTOS_PATH}>")
    free_rtos_fetch()
endif()

find_package(FreeRTOS REQUIRED)
find_package(FreeRTOSPlus REQUIRED)

#changes here
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/demos/projects/ST/b-l475e-iot01a/config")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/b-l475e-iot01a/_deps/freertos-src/FreeRTOS/Source/include")
#include_directories("${CMAKE_CURRENT_SOURCE_DIR}/b-l475e-iot01a/_deps/freertos-src/FreeRTOS/Source/portable/GCC/ARM_CM4_MPU")
#till here 

# Include logging globally
include_directories("${FreeRTOSPlus_PATH}/Source/Utilities/logging")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/demos)
# Add demo
# Add MCU flags, set using board cmake
add_compile_options(${MCU_C_FLAGS})
add_link_options(${MCU_LINKER_FLAGS})

# Set middleware specific path's
set(FREERTOS_DIRECTORY ${FREERTOS_PATH} CACHE STRING "FreeRTOS Path")
set(CONFIG_DIRECTORY ${BOARD_DEMO_CONFIG_PATH} CACHE STRING "Demo config")
set(FREERTOS_PORT_DIRECTORY ${BOARD_DEMO_FREERTOS_PORT_PATH} CACHE STRING "FreeRTOS port")
message(INFO "freertos_port=${FREERTOS_PORT_DIRECTORY}")
message(INFO "freertos_port_path=${BOARD_DEMO_FREERTOS_PORT_PATH}")
message(INFO "freertos_path=${FREERTOS_PATH}")
# Add middleware
set(VT_MIDDLEWARE_RTOS "FREE_RTOS" CACHE STRING "Set VT Middleware to FreeRTOS")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/azure-iot-middleware-freertos)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/Verified-Telemetry verified_telemetry)


