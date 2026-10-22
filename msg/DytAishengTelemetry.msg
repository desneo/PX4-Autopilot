# Aisheng DYT telemetry frame.
#
# Wire format (vendor doc "DYT到无人机的通信协议表"): 104-byte frame, 10 ms,
# header 0x55 0xAA, length byte 0x68 (whole frame, 104 bytes),
# little-endian multi-byte values, checksum byte104 = low 8 bits of sum(bytes 4..103).
#
# Status bytes are kept raw with their vendor bitfield meaning.
# Scaled physical values are converted with the vendor scale factors.
# Target longitude/latitude stay raw int32 (1e-7 deg) to keep full precision.
uint64 timestamp		# publication time

uint8 self_check		# 0x01 ok, 0x02 fault, 0x03 self-testing
uint8 work_status		# bit0 tracking, bit2 defog, bit4-3 video source, bit6-5 algorithm, bit7 RTP
uint8 track_status		# 取work_status的bit0的值，0x0--目标丢失，0x01--跟踪有效
float32 pitch_angle		# 俯仰框架角，pitch frame angle [deg], raw * 0.01
float32 yaw_angle		# 偏航框架角，yaw frame angle [deg], raw * 0.01
int16 gate_x			# tracking gate X [pixel], raw
int16 gate_y			# tracking gate Y [pixel], raw
float32 gyro_pitch_rate		# gyro pitch rate [deg/s], raw * 0.01
float32 gyro_yaw_rate		# gyro yaw rate [deg/s], raw * 0.01
float32 pitch_misalignment	# 俯仰失调角，pitch misalignment angle [deg], raw * 0.01
float32 yaw_misalignment	# 偏航失调角，yaw misalignment angle [deg], raw * 0.01
float32 tracking_confidence	# 跟踪置信度，tracking confidence [0..25.5], raw * 0.1
float32 pitch_los_rate		# 俯仰视线角速度，pitch LOS rate [deg/s], raw * 0.01
float32 yaw_los_rate		# 偏航实现角速度，yaw LOS rate [deg/s], raw * 0.01
uint8 status			# bit0 blind zone, bit1 attack, bit2 level, bit4-3 gyro calib, bit7-5 servo state
uint8 autonomous		# 0 manual, 1 autonomous
uint8[5] target_type		# 目标类别，target 1..5 type: 0 none, 1 tank, 2 vehicle, 3 ship, 4 aircraft, 5 cross
float32[5] target_confidence	# 目标置信度，target 1..5 confidence, raw * 0.01
int32[5] target_lon		# target 1..5 longitude [1e-7 deg], east positive
int32[5] target_lat		# target 1..5 latitude [1e-7 deg], north positive
float32[5] target_alt		# target 1..5 altitude [m], raw * 0.1
float32 laser_range		# laser range [m], raw * 0.1; 0 = invalid
uint8 laser_update		# bit0 updated, bit1-5 positioning condition states
float32 aircraft_pitch		# aircraft pitch [deg], raw * 0.01
float32 aircraft_roll		# aircraft roll [deg], raw * 0.01
float32 aircraft_yaw		# aircraft heading [deg], raw * 0.01
uint8 ip_segment		# DYT IP 192.168.ip_segment.ip_node
uint8 ip_node			# DYT IP 192.168.ip_segment.ip_node
uint32 device_timestamp		# DYT power-on time [ms]
