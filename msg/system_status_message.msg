# @struct SystemStatusMessage
# @zbus system_status wbr_control_system_status_chan
# @enum BootPhase uint8
# @value kBooting 0
# @value kRunning 1
uint8 phase
uint32 active_modules
