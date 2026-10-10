# Start/stop trace command forwarded by serial_probe to the DYT device on TELEM2.
#
# Vendor mapping (see require.txt, 补充需求 2026-10-10):
#   - zhongkeyoucheng: start -> 0x0F (auto lock on detection), stop -> 0x0E (stop tracking)
#   - aisheng:         start -> 0x3F (auto recognition on) + 0x5F (auto lock after recognition),
#                      stop  -> 0x8E (cancel target lock)
# starttrace=true sends the start command, stoptrace=true sends the stop command.
# Messages with both flags set (invalid) or both clear (no action) are ignored.
uint64 timestamp		# publication time
bool starttrace			# request trace start
bool stoptrace			# request trace stop

# Queue a few commands so rapid start/stop publications are not lost.
uint8 ORB_QUEUE_LENGTH = 4
