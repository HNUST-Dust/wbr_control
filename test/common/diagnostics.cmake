# Apply to physical HPM boards; QEMU keeps its harness console.
if(BOARD MATCHES "^(dust-hpm6750|hpm)")
  list(APPEND EXTRA_CONF_FILE "${CMAKE_CURRENT_LIST_DIR}/rtt.conf")
endif()
