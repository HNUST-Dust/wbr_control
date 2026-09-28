cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(onboard_imu_filter_test)

set(AHRS_PARAMS_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/params)
set(AHRS_PARAMS_HEADER ${AHRS_PARAMS_DIR}/ahrs_params_generated.h)
add_custom_command(
  OUTPUT ${AHRS_PARAMS_HEADER}
  COMMAND ${PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_LIST_DIR}/../../tools/generate_params.py
    --input ${CMAKE_CURRENT_LIST_DIR}/../../src/chassis_controller/modules/ahrs/ahrs_params.yaml
    --schema ${CMAKE_CURRENT_LIST_DIR}/../../src/chassis_controller/modules/ahrs/ahrs_params.schema.yaml
    --namespace modules::ahrs_params
    --output ${AHRS_PARAMS_HEADER}
  DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/../../tools/generate_params.py
    ${CMAKE_CURRENT_LIST_DIR}/../../src/chassis_controller/modules/ahrs/ahrs_params.yaml
    ${CMAKE_CURRENT_LIST_DIR}/../../src/chassis_controller/modules/ahrs/ahrs_params.schema.yaml
  VERBATIM
)
add_custom_target(onboard_imu_filter_params DEPENDS ${AHRS_PARAMS_HEADER})
add_dependencies(app onboard_imu_filter_params)

target_sources(app PRIVATE
  src/main.cpp
  ../../src/chassis_controller/modules/ahrs/quaternion_ekf.cpp
)
target_include_directories(app PRIVATE
  ../../src
  ../../../eigen
  ../../../sophus
  ${AHRS_PARAMS_DIR}
)
target_compile_definitions(app PRIVATE
  EIGEN_NO_MALLOC
  EIGEN_DONT_VECTORIZE
)
target_compile_options(app PRIVATE -Wall -Wextra -Werror)
