file(GLOB_RECURSE RM_TEST_ALGORITHMS_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

zephyr_library_named(wbr_algorithms)
zephyr_library_sources(${RM_TEST_ALGORITHMS_SOURCES})

# DSP matrix types are part of a small number of public algorithm templates,
# so consumers of those templates need the DSP headers and implementation.
target_include_directories(wbr_algorithms PUBLIC
  ${PROJECT_SOURCE_DIR}/src
  ${HPM_SDK_DIR}/middleware/hpm_math/nds_dsp
)
target_link_libraries(wbr_algorithms PUBLIC
  ${HPM_SDK_DIR}/middleware/hpm_math/nds_dsp/gcc/${HPM_DSP_LIBRARY}
)
