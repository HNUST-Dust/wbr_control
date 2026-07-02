cmake_minimum_required(VERSION 3.20.0)

set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../hpm_support)
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hi91_imu_test)

target_include_directories(app PRIVATE ../..)

target_sources(app PRIVATE
  src/main.cpp
  ../../protocols/imu/hi91_protocol.cpp
)
