file(GLOB_RECURSE RM_TEST_MODULES_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

# Keep superseded controller sources available for reference without compiling
# them into the firmware.
list(FILTER RM_TEST_MODULES_SOURCES EXCLUDE REGEX "/chassis/legacy/")

target_sources(app PRIVATE ${RM_TEST_MODULES_SOURCES})
