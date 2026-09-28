# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wbr_protocols_test)

target_sources(app PRIVATE
  src/main.cpp
  ../../../src/protocols/imu/hi91_protocol.cpp
  ../../../src/protocols/motors/dji_motor_protocol.cpp
  ../../../src/protocols/ethercat/ethercat_process_data.cpp
  ../../../src/chassis_controller/communication/ethercat_slave_core.cpp
  ../../../src/chassis_controller/communication/ethercat_ssc_bridge.cpp
  ../../../src/protocols/ethercat/ethercat_sii_store.cpp
  ../../../src/protocols/mpc_link/mpc_link.cpp
  ../../../src/protocols/pc_link/pc_link.cpp
  ../../../src/chassis_controller/modules/chassis/ethercat_chassis_port.cpp
)

target_include_directories(app PRIVATE
  ../../../src
)
