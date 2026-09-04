zephyr_library_named(wbr_platform)
target_include_directories(wbr_platform PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(wbr_platform PRIVATE
  wbr_channels
  wbr_protocols
  wbr_scheduling
)

add_subdirectory(board)
add_subdirectory(drivers)
add_subdirectory(storage)
