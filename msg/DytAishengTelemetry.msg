# Aisheng DYT telemetry frame.
#
# Wire format (vendor doc "DYT到无人机的通信协议表"): 104-byte frame, 10 ms,
# header 0x55 0xAA, length byte 0x68 (whole frame, 104 bytes),
# little-endian multi-byte values, checksum byte104 = low 8 bits of sum(bytes 4..103).
#
# Status bytes are kept raw with their vendor bitfield meaning.
# Scaled physical values are converted with the vendor scale factors.
# Target longitude/latitude stay raw int32 (1e-7 deg) to keep full precision.
uint64 timestamp		# time since system start (microseconds)

uint8 self_check		# 0x01 ok, 0x02 fault, 0x03 self-testing
uint8 work_status		# bit0 tracking, bit2 defog, bit4-3 video source, bit6-5 algorithm, bit7 RTP
uint8 track_status		# 取work_status的bit0的值，0x0--目标丢失，0x01--跟踪有效
float32 pitch_angle		# 俯仰框架角，pitch frame angle [deg], raw * 0.01
float32 yaw_angle		# 偏航框架角，yaw frame angle [deg], raw * 0.01
int16 gate_x			# JSON波门中心X原图像素，左上原点向右正；0..32767，失效0
int16 gate_y			# JSON波门中心Y原图像素，左上原点向下正；0..32767，失效0
float32 gyro_pitch_rate		# gyro pitch rate [deg/s], raw * 0.01
float32 gyro_yaw_rate		# gyro yaw rate [deg/s], raw * 0.01
float32 pitch_misalignment	# 俯仰失调角，pitch misalignment angle [deg], raw * 0.01
float32 yaw_misalignment	# 偏航失调角，yaw misalignment angle [deg], raw * 0.01
float32 tracking_confidence	# 跟踪置信度，tracking confidence [0..25.5], raw * 0.1
float32 pitch_los_rate		# 俯仰视线角速度，pitch LOS rate [deg/s], raw * 0.01
float32 yaw_los_rate		# 偏航实现角速度，yaw LOS rate [deg/s], raw * 0.01
uint8 status			# bit0 blind zone, bit1 attack, bit2 level, bit4-3 gyro calib, bit7-5 servo state
uint8 autonomous		# 0 manual, 1 autonomous
uint8[4] target_type		# 目标类别，target 1..4 type: 0 none, 1 tank, 2 vehicle, 3 ship, 4 aircraft, 5 cross
float32[4] target_confidence	# 目标置信度，target 1..4 confidence, raw * 0.01
int32[4] target_lon		# target 1..4 longitude [1e-7 deg], east positive
int32[4] target_lat		# target 1..4 latitude [1e-7 deg], north positive
float32[4] target_alt		# target 1..4 altitude [m], raw * 0.1
uint8 status2			# bit1-0 zoom: 1=1x, 2=2x, 3=4x; bit3-2 seeker mode: 1 background lock, 2 recognition, 3 auto lock
uint16 tracking_gate_w		# 跟踪波门宽度 [pixel], raw
uint16 tracking_gate_h		# 跟踪波门高度 [pixel], raw
float32 pitch_attitude		# 导引头俯仰姿态 [deg], raw * 0.01
float32 yaw_attitude		# 导引头偏航姿态 [deg], raw * 0.01
float32 roll_attitude		# 导引头横滚姿态 [deg], raw * 0.01
uint8 tracking_target_type	# 跟踪目标类型: 0 none, 1 cross, 2 vehicle, 3 person, 4 aircraft, 5 tank
float32 laser_range		# laser range [m], raw * 0.1; 0 = invalid
uint8 laser_update		# bit0 updated, bit1-5 positioning condition states
float32 aircraft_pitch		# aircraft pitch [deg], raw * 0.01
float32 aircraft_roll		# aircraft roll [deg], raw * 0.01
float32 aircraft_yaw		# aircraft heading [deg], raw * 0.01
uint8 ip_segment		# DYT IP 192.168.ip_segment.ip_node
uint8 ip_node			# DYT IP 192.168.ip_segment.ip_node
uint32 device_timestamp		# DYT power-on time [ms]

# 目标相对无人机名义重心的NED视线角速度向量，单位°/s，无倍率、不限幅；不含绕视线自转。
# 旧JSON报文未提供向量时为NaN；新报文明确无效时按约定发送三个0。
float32 los_omega_n		# 北向视线角速度分量，NED N轴，°/s
float32 los_omega_e		# 东向视线角速度分量，NED E轴，°/s
float32 los_omega_d		# 地向视线角速度分量，NED D轴，°/s
