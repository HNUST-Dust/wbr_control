# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(WBR_CONTROL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(HPM_ZEPHYR_DIR ${WBR_CONTROL_ROOT}/../hpm_support)

list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gimbal_controller)

target_sources(app PRIVATE main.cpp)
target_include_directories(app PRIVATE
  ${WBR_CONTROL_ROOT}
  ${WBR_CONTROL_ROOT}/src
)
