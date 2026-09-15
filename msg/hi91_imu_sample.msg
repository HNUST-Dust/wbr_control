# @struct Hi91ImuSample
# @latest latest_hi91_imu_sample
uint32 sequence
uint32 uptime_ms
uint64 precise_timestamp_us
uint32 max_publish_interval_us
uint32 max_sensor_interval_ms
uint32 parse_error_count
uint32 crc_error_count
uint32 publish_gap_with_crc_count
uint32 publish_gap_without_crc_count
uint32 frame_format_error_count
uint32 invalid_sample_count
uint32 rx_drop_count
uint32 rx_stop_count
uint32 rx_buf_rsp_error_count
uint32 system_time_ms
bool valid
float32 roll_deg
float32 pitch_deg
float32 yaw_deg
float32[4] quat
float32[3] gyro_dps
float32[3] accel_g
