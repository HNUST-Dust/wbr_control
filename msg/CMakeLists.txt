file(GLOB_RECURSE WBR_CONTROL_MSG_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

set(WBR_CONTROL_MSG_DEFINITIONS
  ${CMAKE_CURRENT_SOURCE_DIR}/booster_state.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/chassis_realtime_status.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/chassismotors_send_raw.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/chassismotors_feedback_raw.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/gimbal_state.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/hi91_imu_sample.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/onboard_imu_sample.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/pc_auto_aim_command.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/remote_input_state.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/serialservo_send_raw.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/system_status_message.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/oscilloscope_sample.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/uart_raw_frame_queue.msg
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_raw_frame_queue.msg
)
set(WBR_CONTROL_MSG_GENERATED_DIR ${CMAKE_BINARY_DIR}/generated/msg)
set(WBR_CONTROL_MSG_GENERATED_SOURCE
  ${WBR_CONTROL_MSG_GENERATED_DIR}/messages_generated.cpp)
set(WBR_CONTROL_MSG_GENERATED_HEADERS
  ${WBR_CONTROL_MSG_GENERATED_DIR}/booster_state.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/chassis_realtime_status.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/chassismotors_send_raw.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/chassismotors_feedback_raw.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/gimbal_state.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/hi91_imu_sample.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/onboard_imu_sample.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/pc_auto_aim_command.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/remote_input_state.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/serialservo_send_raw.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/system_status_message.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/oscilloscope_sample.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/uart_raw_frame_queue.hpp
  ${WBR_CONTROL_MSG_GENERATED_DIR}/usb_raw_frame_queue.hpp
)

add_custom_command(
  OUTPUT
    ${WBR_CONTROL_MSG_GENERATED_SOURCE}
    ${WBR_CONTROL_MSG_GENERATED_HEADERS}
  COMMAND ${PYTHON_EXECUTABLE} ${WBR_CONTROL_ROOT}/tools/generate_messages.py
    --output-dir ${WBR_CONTROL_MSG_GENERATED_DIR}
    --source-output ${WBR_CONTROL_MSG_GENERATED_SOURCE}
    ${WBR_CONTROL_MSG_DEFINITIONS}
  DEPENDS
    ${WBR_CONTROL_MSG_DEFINITIONS}
    ${WBR_CONTROL_ROOT}/tools/generate_messages.py
  COMMENT "Generating C++ messages from .msg definitions"
  VERBATIM
)

zephyr_library_named(wbr_msg)
zephyr_library_sources(
  ${WBR_CONTROL_MSG_SOURCES}
  ${WBR_CONTROL_MSG_GENERATED_SOURCE}
)
target_include_directories(wbr_msg PUBLIC
  ${WBR_CONTROL_ROOT}
  ${CMAKE_BINARY_DIR}/generated
)
