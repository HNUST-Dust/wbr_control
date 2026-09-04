file(GLOB_RECURSE WBR_CONTROL_PROTOCOLS_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

zephyr_library_named(wbr_protocols)
zephyr_library_sources(${WBR_CONTROL_PROTOCOLS_SOURCES})
target_include_directories(wbr_protocols PUBLIC ${PROJECT_SOURCE_DIR}/src)
