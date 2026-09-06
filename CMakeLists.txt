# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# T2: hpm_support is a project, not manifest repo.
# Compute paths relative to wbr_control location (workspace root).
set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../hpm_support)
set(HPM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk_env/hpm_sdk)
set(HPM_ENV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../sdk_env)
# Eigen：纯头文件线性代数库，位于 workspace 根（../eigen）。
set(EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../eigen)
# Sophus：纯头文件李群库，作为 west project 位于 workspace 根（../sophus）。
set(SOPHUS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sophus)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wbr_control)

# Validate the resolved configuration after Kconfig so UART0 remains reserved
# for the binary oscilloscope stream.
if(NOT CONFIG_UART_ASYNC_API)
  message(FATAL_ERROR
    "The UART0 oscilloscope output requires CONFIG_UART_ASYNC_API=y")
endif()

target_include_directories(app PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
)
target_include_directories(app SYSTEM PRIVATE ${EIGEN_DIR})
if(EXISTS ${SOPHUS_DIR}/sophus/so3.hpp)
  target_include_directories(app SYSTEM PRIVATE ${SOPHUS_DIR})
endif()

if(CONFIG_CHERRYUSB)
  set(CHERRYUSB_CONFIG_DIR ${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
  zephyr_include_directories(${CHERRYUSB_CONFIG_DIR})
  zephyr_include_directories(${HPM_SDK_DIR}/components/usb/device)
endif()

target_sources(app PRIVATE
  src/main.cpp
)

# 模块仅由 main 线程顺序构造，关闭局部静态对象的线程安全初始化守卫。
target_compile_options(app PRIVATE
  -fno-threadsafe-statics
)

add_subdirectory(channels)
add_subdirectory(src/protocols)
add_subdirectory(src/scheduling)
add_subdirectory(src/modules)
add_subdirectory(platform)
add_subdirectory(debug)

# 应用入口只直接依赖模块、通道和平台；协议由模块目标自行声明。
target_link_libraries(app PRIVATE
  wbr_channels
  wbr_modules
  wbr_platform
)

if(TARGET wbr_debug)
  target_link_libraries(app PRIVATE wbr_debug)
endif()
