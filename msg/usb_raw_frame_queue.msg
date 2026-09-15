# @struct UsbRawFrameMessage
# @queue usb_raw_frames 16
# @constant size_t kUsbRawChunkSize 64
uint16 len
uint8[kUsbRawChunkSize] data
