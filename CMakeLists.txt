# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# T2: hpm_support is a project, not manifest repo.
# Compute paths relative to wbr_control location (workspace root).
set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../hpm_support)
set(HPM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk_env/hpm_sdk)
set(HPM_ENV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk_env)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wbr_control)

target_include_directories(app PRIVATE
  .
  algorithms
  ${HPM_SDK_DIR}/middleware/hpm_math/nds_dsp
)

if(CONFIG_RM_TEST_SHELL_MAVLINK_CONTROL)
  target_link_libraries(app PRIVATE mavlink)
endif()

target_link_libraries(app PRIVATE
  ${HPM_SDK_DIR}/middleware/hpm_math/nds_dsp/gcc/libdsp.a
)

if(CONFIG_CHERRYUSB)
  set(CHERRYUSB_CONFIG_DIR ${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
  zephyr_include_directories(${CHERRYUSB_CONFIG_DIR})
  zephyr_include_directories(${HPM_SDK_DIR}/components/usb/device)
endif()

target_sources(app PRIVATE
  src/main.cpp
)

add_subdirectory(modules)
add_subdirectory(algorithms)
add_subdirectory(channels)
add_subdirectory(protocols)
add_subdirectory(debug)
add_subdirectory(platform)
