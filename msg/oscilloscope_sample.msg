# @struct OscilloscopeSample
# @latest latest_oscilloscope_sample
# @constant size_t kOscilloscopeMaxChannels 37
uint32 sequence
uint32 uptime_ms
uint8 channel_count
float32[kOscilloscopeMaxChannels] value
