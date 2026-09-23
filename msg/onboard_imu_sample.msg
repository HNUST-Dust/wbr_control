# @struct OnboardImuSample
# @latest latest_onboard_imu_sample
# @size 40
uint64 timestamp_us
float32[3] euler_deg
float32[3] gyro_rad_s
bool valid
