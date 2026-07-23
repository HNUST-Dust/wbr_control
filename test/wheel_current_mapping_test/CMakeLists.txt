cmake_minimum_required(VERSION 3.20.0)

set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../hpm_support)
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wheel_current_mapping_test)

target_include_directories(app PRIVATE ../..)

target_sources(app PRIVATE
  src/main.cpp
  ../../channels/oscilloscope_sample.cpp
  ../../channels/remote_input_state.cpp
  ../../channels/uart_raw_frame_queue.cpp
  ../../modules/oscilloscope/oscilloscope_module.cpp
  ../../modules/remote_input/remote_input_module.cpp
  ../../protocols/motors/dji_motor_protocol.cpp
  ../../protocols/remote_input/dr16_protocol.cpp
  ../../protocols/remote_input/vt03_protocol.cpp
  ../../protocols/remote_input/wfly_sbus_protocol.cpp
  ../../protocols/telemetry/vofa_protocol.cpp
)
