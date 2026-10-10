# Zhongkeyoucheng (zhongke) DYT periodic telemetry frame.
#
# Wire format (vendor doc "周期性遥测信息"): 32-byte fixed frame, 60 Hz,
# header 0xEE 0x16, little-endian multi-byte values,
# checksum byte31 = low 8 bits of sum(bytes 0..30).
#
# Status bytes are kept raw with their vendor bitfield meaning.
# Scaled physical values are converted with the vendor scale factors.
uint64 timestamp		# publication time

uint8 status1			# status feedback 1: bit7-6 video source, bit5-4 algorithm, bit3 auto hint, bit2 tracking, bit1-0 reserved
uint8 status2			# status feedback 2: bit7 image enhance, bit6 temp zero, bit5 storage, bit4 zero refreshed, bit3 motor, bit2 follow, bit1 electric lock, bit0 laser
uint8 status3			# status feedback 3: bit7-6 large picture, bit5-4 small picture, bit3-0 zoom high nibble
float32 zoom			# zoom factor [ratio], raw * 0.1
float32 offset_x		# target miss distance X [deg], raw * 0.05
float32 offset_y		# target miss distance Y [deg], raw * 0.05
float32 frame_roll		# roll frame angle [deg], raw * 0.01
float32 frame_pitch		# pitch frame angle [deg], raw * 0.01
float32 frame_yaw		# azimuth frame angle [deg], raw * 0.01
float32 roll_rate		# roll frame angular rate [deg/s], raw * 0.01
float32 pitch_rate		# pitch frame angular rate [deg/s], raw * 0.01
float32 yaw_rate		# azimuth frame angular rate [deg/s], raw * 0.01
float32 laser_range		# laser range [m], raw * 0.1; 0 = invalid
uint8 self_check		# self check result: bit7 done, bit2 gyro calibration failed, bit1 encoder/servo error, bit0 image board error
