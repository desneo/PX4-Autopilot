# Custom serial round-trip request/response. Timestamp is publication time.
uint64 timestamp
uint32 sequence
int32 value

# Buffer bursts across the independent serial and processing tasks.
uint8 ORB_QUEUE_LENGTH = 32

# TOPICS serial_probe_request serial_probe_response
