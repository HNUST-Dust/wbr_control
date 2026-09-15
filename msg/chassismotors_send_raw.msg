# @struct ChassisMotorSendRawFrame
# @latest left_wheel_send_raw
# @latest right_wheel_send_raw
# @latest left_b_motor_send_raw
# @latest left_d_motor_send_raw
# @latest right_b_motor_send_raw
# @latest right_d_motor_send_raw
uint8 bus
uint16 can_id
uint8 dlc
uint8[8] data
