# Brand-independent continuous DYT gimbal motion command.
#
# serial_probe converts this physical command to the selected vendor protocol:
#   - zhongkeyoucheng: command 0x24, rate units of 0.1 deg/s
#   - aisheng:         command 0xB1, configurable movement-unit/deg/s gain
#
# Sign convention:
#   yaw:   negative left, positive right
#   pitch: negative down, positive up
# A stop command takes precedence over both rates. Two zero rates also stop.
uint64 timestamp                 # publication time
float32 yaw_rate_deg_s           # requested azimuth rate [deg/s]
float32 pitch_rate_deg_s         # requested elevation rate [deg/s]
bool stop                        # immediately send zero motion and stop keepalive

# Queue rapid direction changes and stop commands so they are not lost.
uint8 ORB_QUEUE_LENGTH = 4
