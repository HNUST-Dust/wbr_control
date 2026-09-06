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

zephyr_library_named(wbr_modules)
zephyr_library_sources(${WBR_CONTROL_MODULES_SOURCES})
target_include_directories(wbr_modules
  PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)
target_include_directories(wbr_modules SYSTEM PRIVATE ${EIGEN_DIR})
if(EXISTS ${SOPHUS_DIR}/sophus/so3.hpp)
  target_include_directories(wbr_modules SYSTEM PRIVATE ${SOPHUS_DIR})
endif()
# 腿部运动学、VMC、车体运动估计和板载 IMU 四元数 EKF 使用 Eigen 固定尺寸矩阵。
target_compile_definitions(wbr_modules PRIVATE
  EIGEN_NO_MALLOC
  EIGEN_DONT_VECTORIZE
)
target_link_libraries(wbr_modules PRIVATE
  wbr_channels
  wbr_platform
  wbr_protocols
  wbr_scheduling
)
