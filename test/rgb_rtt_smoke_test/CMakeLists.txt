cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/../rgb_led_smoke_test/dust-hpm6750.overlay"
)

include(${CMAKE_CURRENT_LIST_DIR}/../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rgb_rtt_smoke_test)

target_sources(app PRIVATE ../rgb_led_smoke_test/src/main.c)
