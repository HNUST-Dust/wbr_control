# @struct RemoteInputState
# @latest latest_remote_state
# @enum RemoteInputSource uint8
# @value kRemoteInputUnknown 0
# @value kRemoteInputDr16 1
# @value kRemoteInputVt03 2
# @value kRemoteInputWfly 3
uint8 source
float32 chassis_x
float32 chassis_rotate
float32 yaw_angle
float32 pitch_angle
float32 leg_length
float32 leg_length_delta
float32 friction_speed
float32 plucker
bool run
bool robot_enable
bool fast_spin
bool supercap
bool auto_aim
bool friction_wheel
bool auth_shoot
bool pc_shoot_control
uint32 sequence
