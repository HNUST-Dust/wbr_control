# @struct ChassisMotorFeedbackRawFrame
# @latest left_wheel_feedback_raw
# @latest right_wheel_feedback_raw
# @latest left_b_motor_feedback_raw
# @latest left_d_motor_feedback_raw
# @latest right_b_motor_feedback_raw
# @latest right_d_motor_feedback_raw
uint64 timestamp_us
uint64 precise_timestamp_us
bool valid
uint8[8] data
