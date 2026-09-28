if(CONFIG_SHELL OR CONFIG_WBR_CONTROL_SHELL_MAVLINK_CONTROL)
  zephyr_library_named(wbr_chassis_debug)
  target_include_directories(wbr_chassis_debug PRIVATE ${WBR_CONTROL_ROOT})

  if(CONFIG_SHELL)
    zephyr_library_sources(shell/shell_banner.cpp)
  endif()

  if(CONFIG_WBR_CONTROL_SHELL_MAVLINK_CONTROL)
    zephyr_library_sources(shell/mavlink_mode_shell.cpp)
    target_link_libraries(wbr_chassis_debug PRIVATE wbr_msg mavlink)
  endif()
endif()
