# @struct UartRawFrameMessage
# @queue referee_uart_raw_messages 32
# @byte_stream remote_input_stream 512
# @constant size_t kUartRawChunkSize 32
uint8 len
uint8[kUartRawChunkSize] data
