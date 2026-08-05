file(GLOB_RECURSE RM_TEST_MODULES_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

# Keep superseded controller sources available for reference without compiling
# them into the firmware.
list(FILTER RM_TEST_MODULES_SOURCES EXCLUDE REGEX "/chassis/legacy/")

zephyr_library_named(wbr_modules)
zephyr_library_sources(${RM_TEST_MODULES_SOURCES})
target_include_directories(wbr_modules
  PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)
target_link_libraries(wbr_modules PRIVATE
  wbr_algorithms
  wbr_channels
  wbr_platform
  wbr_protocols
  wbr_scheduling
)
