file(GLOB_RECURSE RM_TEST_ALGORITHMS_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

zephyr_library_named(wbr_algorithms)
zephyr_library_sources(${RM_TEST_ALGORITHMS_SOURCES})

# 算法层公共依赖：项目头文件。
target_include_directories(wbr_algorithms PUBLIC
  ${PROJECT_SOURCE_DIR}/src
)
# Eigen 作为系统头文件引入，屏蔽其内部告警（如 C++17 下的 std::binder* 弃用告警）。
target_include_directories(wbr_algorithms SYSTEM PUBLIC
  ${EIGEN_DIR}
)
# Eigen 固定尺寸矩阵专用：禁止堆分配、关闭 SIMD（RISC-V 上无 3.4 向量化后端）。
target_compile_definitions(wbr_algorithms PUBLIC
  EIGEN_NO_MALLOC
  EIGEN_DONT_VECTORIZE
)
