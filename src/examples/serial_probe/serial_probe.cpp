/****************************************************************************
 *
 * TELEM2 DYT telemetry parser for zhongkeyoucheng / aisheng devices.
 *
 * The module reads the byte stream on TELEM2 and identifies the vendor by the
 * frame header:
 *   - 0xEE 0x16: zhongkeyoucheng 32-byte periodic telemetry (60 Hz)
 *   - 0x55 0xAA: aisheng 104-byte telemetry (length byte 0x68)
 * Valid frames are checksum-checked and published to
 * dyt_zhongke_telemetry / dyt_aisheng_telemetry.
 *
 * Incoming bytes are kept in a sliding buffer. A header candidate is only
 * consumed when its complete frame has arrived and passed validation; on a
 * checksum or length failure only the first byte is dropped, so a real frame
 * header embedded after a false one is still found.
 *
 * DYT_BRAND selects the vendor so that the port baud rate matches the device
 * (zhongkeyoucheng 115200, aisheng 230400). The header detection itself is
 * independent of the selected brand.
 *
 ****************************************************************************/

#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/dyt_aisheng_telemetry.h>
#include <uORB/topics/dyt_zhongke_telemetry.h>
#include <cerrno>
#include <cinttypes>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr uint8_t kZhongkeSync1 = 0xEE;
constexpr uint8_t kZhongkeSync2 = 0x16;
constexpr uint8_t kZhongkeFrameLength = 32;

constexpr uint8_t kAishengSync1 = 0x55;
constexpr uint8_t kAishengSync2 = 0xAA;
constexpr uint8_t kAishengLengthByte = 0x68;
constexpr uint8_t kAishengFrameLength = 104;

constexpr uint8_t kBrandZhongke = 0;
constexpr uint8_t kBrandAisheng = 1;

int16_t read_s16(const uint8_t *data)
{
	return static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

uint16_t read_u16(const uint8_t *data)
{
	return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

int32_t read_s32(const uint8_t *data)
{
	return static_cast<int32_t>(static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8)
				    | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24));
}

uint32_t read_u32(const uint8_t *data)
{
	return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8)
	       | (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}
} // namespace

class SerialProbe : public ModuleBase<SerialProbe>
{
public:
	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("serial_probe", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
					      2100, (px4_main_t)&run_trampoline, (char *const *)argv);

		if (_task_id < 0) { return PX4_ERROR; }

		return wait_until_running();
	}
	static SerialProbe *instantiate(int argc, char *argv[]) { return new SerialProbe(); }
	static int custom_command(int, char *[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_DESCRIPTION("Receive and publish DYT telemetry from TELEM2.");
		PRINT_MODULE_USAGE_NAME("serial_probe", "example");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}
	int print_status() override
	{
		PX4_INFO("brand=%s(%u) on %s, 8N1", brand_name(), baud_rate(), _device);
		PX4_INFO("zhongke: ok=%" PRIu32 " csum_err=%" PRIu32 "; aisheng: ok=%" PRIu32
			 " csum_err=%" PRIu32 " len_err=%" PRIu32,
			 _zhongke_frames, _zhongke_checksum_errors,
			 _aisheng_frames, _aisheng_checksum_errors, _aisheng_length_errors);
		PX4_INFO("noise_bytes=%" PRIu32 " total_bytes=%" PRIu64, _noise_bytes, _total_bytes);
		return 0;
	}
	void run() override
	{
		int32_t brand = kBrandZhongke;

		if (param_get(param_find("DYT_BRAND"), &brand) != PX4_OK) {
			PX4_WARN("DYT_BRAND unavailable, using zhongkeyoucheng");
			brand = kBrandZhongke;
		}

		if (brand < kBrandZhongke || brand > kBrandAisheng) {
			PX4_WARN("invalid DYT_BRAND %" PRId32 ", using zhongkeyoucheng", brand);
			brand = kBrandZhongke;
		}

		_brand = static_cast<uint8_t>(brand);
		const speed_t speed = (_brand == kBrandAisheng) ? B230400 : B115200;

		_fd = ::open(_device, O_RDONLY | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) { PX4_ERR("open %s: %d", _device, errno); return; }

		termios cfg{};

		if (tcgetattr(_fd, &cfg) != 0) { PX4_ERR("tcgetattr: %d", errno); close_port(); return; }

		cfmakeraw(&cfg);
		cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
		cfg.c_cflag |= CS8 | CLOCAL | CREAD;
		cfsetispeed(&cfg, speed);
		cfsetospeed(&cfg, speed);

		if (tcsetattr(_fd, TCSANOW, &cfg) != 0) { PX4_ERR("tcsetattr: %d", errno); close_port(); return; }

		tcflush(_fd, TCIFLUSH);
		PX4_INFO("parsing %s at %s on %s", brand_name(), baud_name(), _device);

		while (!should_exit()) {
			px4_pollfd_struct_t fdset[1] {};
			fdset[0].fd = _fd;
			fdset[0].events = POLLIN;
			const int ready = px4_poll(fdset, 1, 100);

			if (ready < 0) {
				if (errno == EINTR) { continue; }

				PX4_ERR("poll: %d", errno); break;
			}

			if (fdset[0].revents & (POLLERR | POLLHUP | POLLNVAL)) { PX4_ERR("serial disconnected"); break; }

			if (fdset[0].revents & POLLIN) {
				uint8_t bytes[256];
				const ssize_t count = ::read(_fd, bytes, sizeof(bytes));

				if (count < 0) {
					if (errno == EAGAIN || errno == EINTR) { continue; }

					PX4_ERR("read: %d", errno); break;
				}

				for (ssize_t i = 0; i < count; ++i) { receive(bytes[i]); }
			}
		}

		close_port();
	}
private:
	static const char *brand_name_of(uint8_t brand) { return (brand == kBrandAisheng) ? "aisheng" : "zhongkeyoucheng"; }
	const char *brand_name() const { return brand_name_of(_brand); }
	unsigned baud_rate() const { return (_brand == kBrandAisheng) ? 230400 : 115200; }
	const char *baud_name() const { return (_brand == kBrandAisheng) ? "230400" : "115200"; }
	void close_port()
	{
		if (_fd >= 0) { ::close(_fd); _fd = -1; }
	}
	void receive(uint8_t byte)
	{
		++_total_bytes;

		if (_length >= sizeof(_buffer)) {
			// Keep the newest bytes; at these data rates the buffer only fills
			// up when a header candidate never completes.
			consume(1);
		}

		_buffer[_length++] = byte;
		parse_buffer();
	}
	void consume(unsigned count)
	{
		memmove(_buffer, _buffer + count, _length - count);
		_length -= count;
	}
	void parse_buffer()
	{
		while (true) {
			if (_length < 2) { return; }

			if (_buffer[0] == kZhongkeSync1 && _buffer[1] == kZhongkeSync2) {
				if (_length < kZhongkeFrameLength) { return; }

				if (zhongke_checksum_ok()) {
					publish_zhongke();
					++_zhongke_frames;
					consume(kZhongkeFrameLength);

				} else {
					++_zhongke_checksum_errors;
					consume(1);
				}

				continue;
			}

			if (_buffer[0] == kAishengSync1 && _buffer[1] == kAishengSync2) {
				if (_length < 3) { return; }

				if (_buffer[2] != kAishengLengthByte) {
					++_aisheng_length_errors;
					consume(1);
					continue;
				}

				if (_length < kAishengFrameLength) { return; }

				if (aisheng_checksum_ok()) {
					publish_aisheng();
					++_aisheng_frames;
					consume(kAishengFrameLength);

				} else {
					++_aisheng_checksum_errors;
					consume(1);
				}

				continue;
			}

			++_noise_bytes;
			consume(1);
		}
	}
	bool zhongke_checksum_ok() const
	{
		uint32_t checksum = 0;

		for (unsigned i = 0; i < kZhongkeFrameLength - 1; ++i) { checksum += _buffer[i]; }

		return (checksum & 0xFF) == _buffer[kZhongkeFrameLength - 1];
	}
	bool aisheng_checksum_ok() const
	{
		uint32_t checksum = 0;

		for (unsigned i = 3; i <= kAishengFrameLength - 2; ++i) { checksum += _buffer[i]; }

		return (checksum & 0xFF) == _buffer[kAishengFrameLength - 1];
	}
	void publish_zhongke()
	{
		dyt_zhongke_telemetry_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.status1 = _buffer[2];
		msg.status2 = _buffer[3];
		msg.status3 = _buffer[5];

		const uint16_t zoom_raw = static_cast<uint16_t>(_buffer[4])
					  | static_cast<uint16_t>(static_cast<uint16_t>(_buffer[5] & 0x0F) << 8);
		msg.zoom = zoom_raw * 0.1f;
		msg.offset_x = read_s16(&_buffer[6]) * 0.05f;
		msg.offset_y = read_s16(&_buffer[8]) * 0.05f;
		msg.frame_roll = read_s16(&_buffer[10]) * 0.01f;
		msg.frame_pitch = read_s16(&_buffer[12]) * 0.01f;
		msg.frame_yaw = read_s16(&_buffer[14]) * 0.01f;
		msg.roll_rate = read_s16(&_buffer[20]) * 0.01f;
		msg.pitch_rate = read_s16(&_buffer[22]) * 0.01f;
		msg.yaw_rate = read_s16(&_buffer[24]) * 0.01f;
		msg.laser_range = read_u16(&_buffer[26]) * 0.1f;
		msg.self_check = _buffer[28];

		if (!_zhongke_pub.publish(msg)) { PX4_ERR("publish zhongke telemetry failed"); }
	}
	void publish_aisheng()
	{
		dyt_aisheng_telemetry_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.self_check = _buffer[3];
		msg.work_status = _buffer[4];
		msg.track_status = msg.work_status & 0x01;
		msg.pitch_angle = read_s16(&_buffer[5]) * 0.01f;
		msg.yaw_angle = read_s16(&_buffer[7]) * 0.01f;
		msg.gate_x = read_s16(&_buffer[9]);
		msg.gate_y = read_s16(&_buffer[11]);
		msg.gyro_pitch_rate = read_s16(&_buffer[13]) * 0.01f;
		msg.gyro_yaw_rate = read_s16(&_buffer[15]) * 0.01f;
		msg.pitch_misalignment = read_s16(&_buffer[17]) * 0.01f;
		msg.yaw_misalignment = read_s16(&_buffer[19]) * 0.01f;
		msg.tracking_confidence = _buffer[21] * 0.1f;
		msg.pitch_los_rate = read_s16(&_buffer[22]) * 0.01f;
		msg.yaw_los_rate = read_s16(&_buffer[24]) * 0.01f;
		msg.status = _buffer[26];
		msg.autonomous = _buffer[27];

		for (unsigned i = 0; i < 4; ++i) {
			const uint8_t *target = &_buffer[28 + i * 12];
			msg.target_type[i] = target[0];
			msg.target_confidence[i] = target[1] * 0.01f;
			msg.target_lon[i] = read_s32(&target[2]);
			msg.target_lat[i] = read_s32(&target[6]);
			msg.target_alt[i] = read_u16(&target[10]) * 0.1f;
		}

		msg.status2 = _buffer[76];
		msg.tracking_gate_w = read_u16(&_buffer[77]);
		msg.tracking_gate_h = read_u16(&_buffer[79]);
		msg.pitch_attitude = read_s16(&_buffer[81]) * 0.01f;
		msg.yaw_attitude = read_s16(&_buffer[83]) * 0.01f;
		msg.roll_attitude = read_s16(&_buffer[85]) * 0.01f;
		msg.tracking_target_type = _buffer[87];
		msg.laser_range = read_u16(&_buffer[88]) * 0.1f;
		msg.laser_update = _buffer[90];
		msg.aircraft_pitch = read_s16(&_buffer[91]) * 0.01f;
		msg.aircraft_roll = read_s16(&_buffer[93]) * 0.01f;
		msg.aircraft_yaw = read_u16(&_buffer[95]) * 0.01f;
		msg.ip_segment = _buffer[97];
		msg.ip_node = _buffer[98];
		msg.device_timestamp = read_u32(&_buffer[99]);

		if (!_aisheng_pub.publish(msg)) { PX4_ERR("publish aisheng telemetry failed"); }
	}
	char _device[16] {"/dev/ttyS3"};
	int _fd{-1};
	uint8_t _brand{kBrandZhongke};
	uint8_t _buffer[192] {};
	unsigned _length{0};
	uint32_t _zhongke_frames{0};
	uint32_t _zhongke_checksum_errors{0};
	uint32_t _aisheng_frames{0};
	uint32_t _aisheng_checksum_errors{0};
	uint32_t _aisheng_length_errors{0};
	uint32_t _noise_bytes{0};
	uint64_t _total_bytes{0};
	uORB::Publication<dyt_zhongke_telemetry_s> _zhongke_pub{ORB_ID(dyt_zhongke_telemetry)};
	uORB::Publication<dyt_aisheng_telemetry_s> _aisheng_pub{ORB_ID(dyt_aisheng_telemetry)};
};

extern "C" __EXPORT int serial_probe_main(int argc, char *argv[])
{
	return SerialProbe::main(argc, argv);
}
