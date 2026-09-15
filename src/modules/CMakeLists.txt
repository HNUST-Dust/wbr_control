set(WBR_CONTROL_MODULES_SOURCES)

function(wbr_add_module_sources config_symbol directory)
  if(${config_symbol})
    file(GLOB_RECURSE module_sources CONFIGURE_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/${directory}/*.c
      ${CMAKE_CURRENT_SOURCE_DIR}/${directory}/*.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/${directory}/*.cpp
    )
    # Keep superseded controller sources available for reference without
    # compiling them into the firmware.
    list(FILTER module_sources EXCLUDE REGEX "/legacy/")
    # MPC is retained as an application-owned implementation for a future
    # CPU1 or host transport, but CPU0's chassis module remains LQR-only.
    list(FILTER module_sources EXCLUDE REGEX "/chassis/mpc/")
    set(WBR_CONTROL_MODULES_SOURCES
      ${WBR_CONTROL_MODULES_SOURCES} ${module_sources}
      PARENT_SCOPE
    )
  endif()
endfunction()

wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_AHRS ahrs)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_CHASSIS chassis)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_HI91_IMU imu)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_OSCILLOSCOPE oscilloscope)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_REFEREE referee)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_REMOTE_INPUT remote_input)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_SYS_STATE sys_state)
wbr_add_module_sources(CONFIG_WBR_CONTROL_MODULE_SDLOG sdlog)

zephyr_library_named(wbr_modules)
zephyr_library_sources(${WBR_CONTROL_MODULES_SOURCES})

if(CONFIG_WBR_CONTROL_MODULE_CHASSIS)
  wbr_generate_params(
    wbr_modules chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/chassis_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis/chassis_params.schema.yaml
    modules::chassis_params)
endif()

if(CONFIG_WBR_CONTROL_MODULE_AHRS)
  wbr_generate_params(
    wbr_modules ahrs
    ${CMAKE_CURRENT_SOURCE_DIR}/ahrs/ahrs_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/ahrs/ahrs_params.schema.yaml
    modules::ahrs_params)
endif()

if(CONFIG_WBR_CONTROL_MODULE_HI91_IMU)
  wbr_generate_params(
    wbr_modules hi91_imu
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/hi91_imu_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/imu/hi91_imu_params.schema.yaml
    modules::hi91_imu_params)
endif()

if(CONFIG_WBR_CONTROL_MODULE_REMOTE_INPUT)
  wbr_generate_params(
    wbr_modules remote_input
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_input/remote_input_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_input/remote_input_params.schema.yaml
    modules::remote_input_params)
endif()

if(CONFIG_WBR_CONTROL_MODULE_OSCILLOSCOPE)
  wbr_generate_params(
    wbr_modules oscilloscope
    ${CMAKE_CURRENT_SOURCE_DIR}/oscilloscope/oscilloscope_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/oscilloscope/oscilloscope_params.schema.yaml
    modules::oscilloscope_params)
endif()

if(CONFIG_WBR_CONTROL_MODULE_SYS_STATE)
  wbr_generate_params(
    wbr_modules sys_state
    ${CMAKE_CURRENT_SOURCE_DIR}/sys_state/sys_state_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/sys_state/sys_state_params.schema.yaml
    modules::sys_state_params)
endif()

if(CONFIG_WBR_CONTROL_MODULE_SDLOG)
  wbr_generate_params(
    wbr_modules sdlog
    ${CMAKE_CURRENT_SOURCE_DIR}/sdlog/sdlog_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/sdlog/sdlog_params.schema.yaml
    modules::sdlog_params)
endif()

target_include_directories(wbr_modules
  PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)
target_include_directories(wbr_modules SYSTEM PRIVATE ${EIGEN_DIR})
target_include_directories(wbr_modules SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/../fatfs/include)
if(EXISTS ${SOPHUS_DIR}/sophus/so3.hpp)
  target_include_directories(wbr_modules SYSTEM PRIVATE ${SOPHUS_DIR})
endif()
# 腿部运动学、VMC、车体运动估计和板载 IMU 四元数 EKF 使用 Eigen 固定尺寸矩阵。
target_compile_definitions(wbr_modules PRIVATE
  EIGEN_NO_MALLOC
  EIGEN_DONT_VECTORIZE
)
target_link_libraries(wbr_modules PRIVATE
  wbr_msg
  wbr_platform
  wbr_protocols
  wbr_scheduling
)
