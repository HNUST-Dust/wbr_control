if(CONFIG_SHELL OR CONFIG_RM_TEST_SHELL_MAVLINK_CONTROL)
  zephyr_library_named(wbr_debug)
  target_include_directories(wbr_debug PRIVATE ${PROJECT_SOURCE_DIR})

  if(CONFIG_SHELL)
    zephyr_library_sources(shell/shell_banner.cpp)
  endif()

  if(CONFIG_RM_TEST_SHELL_MAVLINK_CONTROL)
    zephyr_library_sources(shell/mavlink_mode_shell.cpp)
    target_link_libraries(wbr_debug PRIVATE wbr_channels mavlink)
  endif()
endif()
