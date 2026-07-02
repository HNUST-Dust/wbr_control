cmake_minimum_required(VERSION 3.20.0)

set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../hpm_support)
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(leg_feedforward_test)

target_include_directories(app PRIVATE ../..)

target_sources(app PRIVATE
  src/main.cpp
  ../../protocols/motors/dm_motor_protocol.cpp
  ../../modules/chassis/leg_kinematics.cc
)
