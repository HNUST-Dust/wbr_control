file(GLOB_RECURSE RM_TEST_CHANNELS_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

zephyr_library_named(wbr_channels)
zephyr_library_sources(${RM_TEST_CHANNELS_SOURCES})
target_include_directories(wbr_channels PUBLIC ${PROJECT_SOURCE_DIR})
