cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(onboard_imu_filter_test)

target_sources(app PRIVATE
  src/main.cpp
  ../../src/modules/ahrs/quaternion_ekf.cpp
)
target_include_directories(app PRIVATE
  ../../src
  ../../../eigen
  ../../../sophus
)
target_compile_definitions(app PRIVATE
  EIGEN_NO_MALLOC
  EIGEN_DONT_VECTORIZE
)
target_compile_options(app PRIVATE -Wall -Wextra -Werror)
