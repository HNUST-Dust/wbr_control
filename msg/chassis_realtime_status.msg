# @struct ChassisRealtimeStatus
# @latest latest_chassis_realtime_status
# @enum ChassisImuSource uint8
# @value kHi91 0
# @value kOnboardEkf 2
uint32 sequence
uint32 uptime_ms
uint32 deadline_miss_count
uint32 loop_execution_us
uint32 max_loop_execution_us
uint32 loop_period_us
uint32 min_loop_period_us
uint32 max_loop_period_us
uint32 stack_unused_bytes
uint64 imu_age_us
uint8 imu_source
bool imu_fresh
