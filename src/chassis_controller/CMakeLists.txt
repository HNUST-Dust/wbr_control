# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(WBR_CONTROL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# T2: hpm_support is a project, not manifest repo.
# Compute paths relative to wbr_control location (workspace root).
set(HPM_ZEPHYR_DIR ${WBR_CONTROL_ROOT}/../hpm_support)
set(HPM_SDK_DIR ${WBR_CONTROL_ROOT}/../../sdk_env/hpm_sdk)
set(HPM_ENV_DIR ${WBR_CONTROL_ROOT}/../../sdk_env)
# Eigen：纯头文件线性代数库，位于 workspace 根（../eigen）。
set(EIGEN_DIR ${WBR_CONTROL_ROOT}/../eigen)
# Sophus：纯头文件李群库，作为 west project 位于 workspace 根（../sophus）。
set(SOPHUS_DIR ${WBR_CONTROL_ROOT}/../sophus)
set(FATFS_DIR ${WBR_CONTROL_ROOT}/../fatfs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(chassis_controller)

# Generate validated constexpr parameter headers from value/schema YAML pairs.
# Targets throughout the repository use this one helper and one Python tool.
function(wbr_generate_params target name values schema cpp_namespace)
  set(generated_dir ${CMAKE_BINARY_DIR}/generated/params)
  set(generated_header ${generated_dir}/${name}_params_generated.h)
  set(generator ${WBR_CONTROL_ROOT}/tools/generate_params.py)

  add_custom_command(
    OUTPUT ${generated_header}
    COMMAND ${PYTHON_EXECUTABLE} ${generator}
      --input ${values}
      --schema ${schema}
      --namespace ${cpp_namespace}
      --output ${generated_header}
    DEPENDS ${values} ${schema} ${generator}
    COMMENT "Generating validated ${name} parameters"
    VERBATIM
  )
  add_custom_target(wbr_${name}_params DEPENDS ${generated_header})
  add_dependencies(${target} wbr_${name}_params)
  target_include_directories(${target} PRIVATE ${generated_dir})
endfunction()

# Validate the resolved configuration after Kconfig so UART0 remains reserved
# for the binary oscilloscope stream.
if(NOT CONFIG_UART_ASYNC_API)
  message(FATAL_ERROR
    "The UART0 oscilloscope output requires CONFIG_UART_ASYNC_API=y")
endif()

target_include_directories(app PRIVATE
  ${WBR_CONTROL_ROOT}
  ${WBR_CONTROL_ROOT}/src
)
target_include_directories(app SYSTEM PRIVATE ${EIGEN_DIR})
target_include_directories(app SYSTEM PRIVATE ${FATFS_DIR}/include)
if(EXISTS ${SOPHUS_DIR}/sophus/so3.hpp)
  target_include_directories(app SYSTEM PRIVATE ${SOPHUS_DIR})
endif()

if(CONFIG_CHERRYUSB)
  set(CHERRYUSB_CONFIG_DIR ${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
  zephyr_include_directories(${CHERRYUSB_CONFIG_DIR})
  zephyr_include_directories(${HPM_SDK_DIR}/components/usb/device)
endif()

target_sources(app PRIVATE
  main.cpp
)

# 模块仅由 main 线程顺序构造，关闭局部静态对象的线程安全初始化守卫。
target_compile_options(app PRIVATE
  -fno-threadsafe-statics
)

add_subdirectory(${WBR_CONTROL_ROOT}/msg ${CMAKE_BINARY_DIR}/wbr_msg)
add_subdirectory(${WBR_CONTROL_ROOT}/src/protocols ${CMAKE_BINARY_DIR}/wbr_protocols)
add_subdirectory(${WBR_CONTROL_ROOT}/src/scheduling ${CMAKE_BINARY_DIR}/wbr_scheduling)
add_subdirectory(${WBR_CONTROL_ROOT}/platform ${CMAKE_BINARY_DIR}/wbr_platform)
include(${CMAKE_CURRENT_SOURCE_DIR}/modules.cmake)
add_subdirectory(${WBR_CONTROL_ROOT}/debug ${CMAKE_BINARY_DIR}/wbr_debug)

# 应用入口只直接依赖模块、通道和平台；协议由模块目标自行声明。
target_link_libraries(app PRIVATE
  wbr_msg
  wbr_modules
  wbr_platform
)

if(TARGET wbr_debug)
  target_link_libraries(app PRIVATE wbr_debug)
endif()
