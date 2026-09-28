# SPDX-License-Identifier: Apache-2.0

# The gimbal application owns this platform boundary. Keep it as an interface
# target until the first gimbal-specific board adapter or device driver lands.
add_library(wbr_gimbal_platform INTERFACE)
target_include_directories(wbr_gimbal_platform INTERFACE
  ${WBR_CONTROL_ROOT}/src
)
