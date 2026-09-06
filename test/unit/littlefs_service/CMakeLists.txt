# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(littlefs_service_test)

target_sources(app PRIVATE
  src/main.cpp
  ../../../platform/storage/filesystem/littlefs_service.cpp
)

target_include_directories(app PRIVATE
  ../../..
  ../../../../littlefs
)

# Host unit tests do not have an application devicetree partition. The value is
# only forwarded to the mocked filesystem API in this test binary.
target_compile_definitions(app PRIVATE
  WBR_CONTROL_STORAGE_PARTITION_ID=7
  CONFIG_FS_LITTLEFS_READ_SIZE=16
  CONFIG_FS_LITTLEFS_PROG_SIZE=16
  CONFIG_FS_LITTLEFS_CACHE_SIZE=64
  CONFIG_FS_LITTLEFS_LOOKAHEAD_SIZE=32
)
