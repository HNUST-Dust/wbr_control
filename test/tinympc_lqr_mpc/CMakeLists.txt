cmake_minimum_required(VERSION 3.20)
project(wbr_tinympc_lqr_mpc_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TINYMPC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../TinyMPC)
set(TINYMPC_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(${TINYMPC_DIR}
                 ${CMAKE_CURRENT_BINARY_DIR}/tinympc EXCLUDE_FROM_ALL)

add_executable(wbr_tinympc_lqr_mpc_test src/main.cpp)
target_link_libraries(wbr_tinympc_lqr_mpc_test PRIVATE tinympcstatic)
target_include_directories(wbr_tinympc_lqr_mpc_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../src
)
target_compile_options(wbr_tinympc_lqr_mpc_test PRIVATE -O3)
target_compile_definitions(wbr_tinympc_lqr_mpc_test PRIVATE EIGEN_RUNTIME_NO_MALLOC)

# This target deliberately does not link tinympcstatic. It proves that the
# fixed-size online kernel and offline-generated cache are self-contained.
add_executable(wbr_tinympc_static_smoke src/static_only_smoke.cpp)
target_include_directories(wbr_tinympc_static_smoke PRIVATE
  ${TINYMPC_DIR}/include/Eigen
  ${TINYMPC_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/../../src
  ${CMAKE_CURRENT_LIST_DIR}/src
)
target_compile_options(wbr_tinympc_static_smoke PRIVATE -O3)
target_compile_definitions(wbr_tinympc_static_smoke PRIVATE EIGEN_RUNTIME_NO_MALLOC)

add_executable(wbr_tinympc_production_sweep src/production_sweep.cpp)
target_include_directories(wbr_tinympc_production_sweep PRIVATE
  ${TINYMPC_DIR}/include/Eigen
  ${CMAKE_CURRENT_LIST_DIR}/../../src
  ${CMAKE_CURRENT_LIST_DIR}/src
)
target_compile_options(wbr_tinympc_production_sweep PRIVATE -O3)
target_compile_definitions(wbr_tinympc_production_sweep PRIVATE EIGEN_NO_MALLOC)

enable_testing()
add_test(NAME wbr_tinympc_static_smoke COMMAND wbr_tinympc_static_smoke)
add_test(NAME wbr_tinympc_production_sweep COMMAND wbr_tinympc_production_sweep)
