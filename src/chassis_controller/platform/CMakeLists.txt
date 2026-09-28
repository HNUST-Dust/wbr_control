zephyr_library_named(wbr_chassis_platform)
target_include_directories(wbr_chassis_platform PUBLIC ${WBR_CONTROL_ROOT}/src)
target_link_libraries(wbr_chassis_platform PRIVATE
  wbr_msg
  wbr_protocols
)

add_subdirectory(board)
add_subdirectory(drivers)
add_subdirectory(storage)
