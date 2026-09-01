cmake_minimum_required(VERSION 3.20.0)

if(NOT DEFINED DTC_OVERLAY_FILE)
  set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/dust-hpm6750.overlay")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(spi_test)

target_sources(app PRIVATE src/main.c)
