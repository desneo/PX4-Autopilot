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
#include <uORB/topics/dyt_gimbal_rate_command.h>
#include <uORB/topics/dyt_trace_command.h>
#include <uORB/topics/dyt_zhongke_telemetry.h>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
constexpr float kStoppedRateEpsilon = 0.0001f;

float clamp_float(float value, float minimum, float maximum)
{
	return value < minimum ? minimum : (value > maximum ? maximum : value);
}

float move_towards(float current, float target, float maximum_delta)
{
	if (target > current) { return current + fminf(target - current, maximum_delta); }

	if (target < current) { return current - fminf(current - target, maximum_delta); }

	return target;
}

void write_s16(uint8_t *data, int16_t value)
{
	const uint16_t raw = static_cast<uint16_t>(value);
	data[0] = raw & 0xFF;
	data[1] = (raw >> 8) & 0xFF;
}

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
	static int custom_command(int argc, char *argv[])
	{
		if (argc >= 2 && strcmp(argv[0], "trace") == 0) {
			if (strcmp(argv[1], "start") == 0) { return publish_trace_command(true); }

			if (strcmp(argv[1], "stop") == 0) { return publish_trace_command(false); }

			return print_usage("trace expects 'start' or 'stop'");
		}

		if (argc >= 2 && strcmp(argv[0], "gimbal") == 0) {
			if (argc == 2 && strcmp(argv[1], "stop") == 0) { return publish_gimbal_command(0.f, 0.f, true); }

			if (argc == 3) {
				float yaw_rate = 0.f;
				float pitch_rate = 0.f;

				if (!parse_float_arg(argv[1], yaw_rate) || !parse_float_arg(argv[2], pitch_rate)) {
					return print_usage("gimbal rates must be finite numbers");
				}

				return publish_gimbal_command(yaw_rate, pitch_rate, false);
			}

			return print_usage("gimbal expects '<yaw_deg_s> <pitch_deg_s>' or 'stop'");
		}

		return print_usage("unknown command");
	}
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_DESCRIPTION("Parse DYT telemetry and translate common DYT control commands on TELEM2.");
		PRINT_MODULE_USAGE_NAME("serial_probe", "example");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_ARG("trace start|stop", "publish a dyt_trace_command", true);
		PRINT_MODULE_USAGE_ARG("gimbal <yaw_deg_s> <pitch_deg_s>|stop",
				       "publish a continuous dyt_gimbal_rate_command", true);
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
		PX4_INFO("trace commands: start=%" PRIu32 " stop=%" PRIu32, _trace_starts, _trace_stops);
		PX4_INFO("gimbal: active=%s target=%.2f/%.2f deg/s current=%.2f/%.2f deg/s",
			 _gimbal_active ? "yes" : "no", (double)_target_yaw_rate, (double)_target_pitch_rate,
			 (double)_current_yaw_rate, (double)_current_pitch_rate);
		PX4_INFO("gimbal config: max=%.1f deg/s slew=%.1f deg/s^2 interval=%" PRIu32
			 " ms aisheng_gain=%.2f; commands=%" PRIu32 " stops=%" PRIu32 " tx=%" PRIu32,
			 (double)_rate_max, (double)_rate_slew, (uint32_t)(_command_interval_us / 1000),
			 (double)_aisheng_gain, _gimbal_commands, _gimbal_stops, _gimbal_transmissions);
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
		load_gimbal_parameters();
		const int command_poll_timeout_ms = static_cast<int>(_command_interval_us / 1000);
		const int poll_timeout_ms = command_poll_timeout_ms < 100 ? command_poll_timeout_ms : 100;
		_start_time = hrt_absolute_time();

		_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) { PX4_ERR("open %s: %d", _device, errno); return; }

		termios cfg{};

		if (tcgetattr(_fd, &cfg) != 0) { PX4_ERR("tcgetattr: %d", errno); close_port(); return; }

		cfmakeraw(&cfg);
		cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
		cfg.c_cflag |= CS8 | CLOCAL | CREAD;
		cfsetispeed(&cfg, speed);
		cfsetospeed(&cfg, speed);

		if (tcsetattr(_fd, TCSANOW, &cfg) != 0) { PX4_ERR("tcsetattr: %d", errno); close_port(); return; }

		_trace_fd = orb_subscribe(ORB_ID(dyt_trace_command));

		if (_trace_fd < 0) { PX4_ERR("subscribe dyt_trace_command failed: %d", errno); close_port(); return; }

		_gimbal_fd = orb_subscribe(ORB_ID(dyt_gimbal_rate_command));

		if (_gimbal_fd < 0) {
			PX4_ERR("subscribe dyt_gimbal_rate_command failed: %d", errno); close_port(); return;
		}

		tcflush(_fd, TCIOFLUSH);
		PX4_INFO("parsing %s at %s on %s", brand_name(), baud_name(), _device);

		while (!should_exit()) {
			px4_pollfd_struct_t fdset[3] {};
			fdset[0].fd = _fd;
			fdset[0].events = POLLIN;
			fdset[1].fd = _trace_fd;
			fdset[1].events = POLLIN;
			fdset[2].fd = _gimbal_fd;
			fdset[2].events = POLLIN;
			const int ready = px4_poll(fdset, 3, poll_timeout_ms);

			if (ready < 0) {
				if (errno == EINTR) { continue; }

				PX4_ERR("poll: %d", errno); break;
			}

			if (fdset[0].revents & (POLLERR | POLLHUP | POLLNVAL)) { PX4_ERR("serial disconnected"); break; }

			if (fdset[1].revents & POLLIN) { handle_trace_commands(); }

			if (fdset[2].revents & POLLIN) { handle_gimbal_commands(); }

			if (fdset[0].revents & POLLIN) {
				uint8_t bytes[256];
				const ssize_t count = ::read(_fd, bytes, sizeof(bytes));

				if (count < 0) {
					if (errno == EAGAIN || errno == EINTR) { continue; }

					PX4_ERR("read: %d", errno); break;
				}

				for (ssize_t i = 0; i < count; ++i) { receive(bytes[i]); }
			}

			update_gimbal_motion();
		}

		close_port();
	}
private:
	static const char *brand_name_of(uint8_t brand) { return (brand == kBrandAisheng) ? "aisheng" : "zhongkeyoucheng"; }
	const char *brand_name() const { return brand_name_of(_brand); }
	unsigned baud_rate() const { return (_brand == kBrandAisheng) ? 230400 : 115200; }
	const char *baud_name() const { return (_brand == kBrandAisheng) ? "230400" : "115200"; }
	static bool parse_float_arg(const char *text, float &value)
	{
		if (text == nullptr || text[0] == '\0') { return false; }

		char *end = nullptr;
		errno = 0;
		value = strtof(text, &end);
		return errno == 0 && end != text && *end == '\0' && std::isfinite(value);
	}
	void load_gimbal_parameters()
	{
		float value = 0.f;

		if (param_get(param_find("DYT_RATE_MAX"), &value) == PX4_OK && std::isfinite(value)) {
			_rate_max = clamp_float(value, 0.1f, 100.f);
		}

		if (param_get(param_find("DYT_RATE_SLEW"), &value) == PX4_OK && std::isfinite(value)) {
			_rate_slew = clamp_float(value, 0.f, 500.f);
		}

		int32_t interval_ms = 100;

		if (param_get(param_find("DYT_CMD_INT"), &interval_ms) == PX4_OK) {
			if (interval_ms < 20) { interval_ms = 20; }

			if (interval_ms > 1000) { interval_ms = 1000; }
		}

		_command_interval_us = static_cast<hrt_abstime>(interval_ms) * 1000;

		if (param_get(param_find("DYT_AS_GAIN"), &value) == PX4_OK && std::isfinite(value)) {
			_aisheng_gain = clamp_float(value, 0.1f, 100.f);
		}
	}
	static int publish_trace_command(bool start)
	{
		SerialProbe *instance = get_instance();

		if (instance == nullptr) { PX4_ERR("serial_probe is not running"); return PX4_ERROR; }

		dyt_trace_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.starttrace = start;
		command.stoptrace = !start;

		if (!instance->_trace_pub.publish(command)) { PX4_ERR("publish dyt_trace_command failed"); return PX4_ERROR; }

		PX4_INFO("published dyt_trace_command (%s)", start ? "start" : "stop");
		return PX4_OK;
	}
	static int publish_gimbal_command(float yaw_rate, float pitch_rate, bool stop)
	{
		SerialProbe *instance = get_instance();

		if (instance == nullptr) { PX4_ERR("serial_probe is not running"); return PX4_ERROR; }

		dyt_gimbal_rate_command_s command{};
		command.timestamp = hrt_absolute_time();
		command.yaw_rate_deg_s = yaw_rate;
		command.pitch_rate_deg_s = pitch_rate;
		command.stop = stop;

		if (!instance->_gimbal_pub.publish(command)) {
			PX4_ERR("publish dyt_gimbal_rate_command failed"); return PX4_ERROR;
		}

		PX4_INFO("published dyt_gimbal_rate_command (yaw=%.2f pitch=%.2f stop=%s)",
			 (double)yaw_rate, (double)pitch_rate, stop ? "yes" : "no");
		return PX4_OK;
	}
	void close_port()
	{
		if (_trace_fd >= 0) { orb_unsubscribe(_trace_fd); _trace_fd = -1; }
		if (_gimbal_fd >= 0) { orb_unsubscribe(_gimbal_fd); _gimbal_fd = -1; }

		if (_fd >= 0) { ::close(_fd); _fd = -1; }
	}
	void handle_trace_commands()
	{
		while (!should_exit()) {
			bool updated = false;

			if (orb_check(_trace_fd, &updated) != PX4_OK || !updated) { break; }

			dyt_trace_command_s command{};

			if (orb_copy(ORB_ID(dyt_trace_command), _trace_fd, &command) != PX4_OK) {
				PX4_ERR("copy dyt_trace_command failed"); break;
			}

			// uORB redelivers the retained last sample to a new subscriber.
			// Only act on commands published after this module start.
			if (command.timestamp < _start_time) { continue; }

			dispatch_trace_command(command.starttrace, command.stoptrace);
		}
	}
	void dispatch_trace_command(bool starttrace, bool stoptrace)
	{
		if (starttrace == stoptrace) {
			if (starttrace) { PX4_WARN("dyt_trace_command invalid: starttrace and stoptrace both set"); }

			return;
		}

		bool sent = false;

		if (_brand == kBrandAisheng) {
			if (starttrace) {
				sent = send_aisheng_command(0x3F) && send_aisheng_command(0x5F);

			} else {
				sent = send_aisheng_command(0x8E);
			}

		} else {
			sent = send_zhongke_command(starttrace ? 0x0F : 0x0E);
		}

		if (!sent) { PX4_ERR("trace command TX failed"); return; }

		if (starttrace) {
			++_trace_starts;
			PX4_INFO("trace start sent (%s)", brand_name());

		} else {
			++_trace_stops;
			PX4_INFO("trace stop sent (%s)", brand_name());
		}
	}
	void handle_gimbal_commands()
	{
		while (!should_exit()) {
			bool updated = false;

			if (orb_check(_gimbal_fd, &updated) != PX4_OK || !updated) { break; }

			dyt_gimbal_rate_command_s command{};

			if (orb_copy(ORB_ID(dyt_gimbal_rate_command), _gimbal_fd, &command) != PX4_OK) {
				PX4_ERR("copy dyt_gimbal_rate_command failed"); break;
			}

			// Do not replay a retained motion command after a module restart.
			if (command.timestamp < _start_time) { continue; }

			dispatch_gimbal_command(command);
		}
	}
	void dispatch_gimbal_command(const dyt_gimbal_rate_command_s &command)
	{
		if (!std::isfinite(command.yaw_rate_deg_s) || !std::isfinite(command.pitch_rate_deg_s)) {
			PX4_WARN("dyt_gimbal_rate_command rejected: non-finite rate");
			return;
		}

		const bool zero_rates = fabsf(command.yaw_rate_deg_s) < kStoppedRateEpsilon
					&& fabsf(command.pitch_rate_deg_s) < kStoppedRateEpsilon;

		if (command.stop || zero_rates) {
			stop_gimbal();
			return;
		}

		_target_yaw_rate = clamp_float(command.yaw_rate_deg_s, -_rate_max, _rate_max);
		_target_pitch_rate = clamp_float(command.pitch_rate_deg_s, -_rate_max, _rate_max);

		if (fabsf(_target_yaw_rate - command.yaw_rate_deg_s) > kStoppedRateEpsilon
		    || fabsf(_target_pitch_rate - command.pitch_rate_deg_s) > kStoppedRateEpsilon) {
			PX4_WARN("gimbal rate clamped to +/-%.1f deg/s", (double)_rate_max);
		}

		const hrt_abstime now = hrt_absolute_time();
		const bool was_active = _gimbal_active;
		_gimbal_active = true;
		++_gimbal_commands;

		if (!was_active || _last_slew_time == 0) {
			_last_slew_time = now > _command_interval_us ? now - _command_interval_us : 0;
		}

		advance_gimbal_rates(now);

		if (!send_gimbal_rate(_current_yaw_rate, _current_pitch_rate)) {
			PX4_ERR("gimbal command TX failed");
		}
	}
	void stop_gimbal()
	{
		_target_yaw_rate = 0.f;
		_target_pitch_rate = 0.f;
		_current_yaw_rate = 0.f;
		_current_pitch_rate = 0.f;
		_gimbal_active = false;
		_last_slew_time = hrt_absolute_time();
		++_gimbal_stops;

		if (!send_gimbal_rate(0.f, 0.f)) { PX4_ERR("gimbal stop TX failed"); }
	}
	void advance_gimbal_rates(hrt_abstime now)
	{
		if (_rate_slew <= 0.f) {
			_current_yaw_rate = _target_yaw_rate;
			_current_pitch_rate = _target_pitch_rate;
			_last_slew_time = now;
			return;
		}

		const float elapsed_s = static_cast<float>(now - _last_slew_time) * 1e-6f;
		const float maximum_delta = _rate_slew * elapsed_s;
		_current_yaw_rate = move_towards(_current_yaw_rate, _target_yaw_rate, maximum_delta);
		_current_pitch_rate = move_towards(_current_pitch_rate, _target_pitch_rate, maximum_delta);
		_last_slew_time = now;
	}
	void update_gimbal_motion()
	{
		if (!_gimbal_active) { return; }

		const hrt_abstime now = hrt_absolute_time();

		if (_last_gimbal_tx != 0 && now - _last_gimbal_tx < _command_interval_us) { return; }

		advance_gimbal_rates(now);

		if (!send_gimbal_rate(_current_yaw_rate, _current_pitch_rate)) {
			PX4_ERR("gimbal keepalive TX failed");
		}
	}
	bool send_gimbal_rate(float yaw_rate, float pitch_rate)
	{
		bool sent = false;

		if (_brand == kBrandAisheng) {
			const int16_t yaw_movement = static_cast<int16_t>(lroundf(clamp_float(yaw_rate * _aisheng_gain, -100.f, 100.f)));
			const int16_t pitch_movement = static_cast<int16_t>(lroundf(clamp_float(pitch_rate * _aisheng_gain, -100.f, 100.f)));
			sent = send_aisheng_command(0xB1, yaw_movement, pitch_movement);

		} else {
			const int16_t yaw_rate_raw = static_cast<int16_t>(lroundf(yaw_rate * 10.f));
			const int16_t pitch_rate_raw = static_cast<int16_t>(lroundf(pitch_rate * 10.f));
			sent = send_zhongke_command(0x24, yaw_rate_raw, pitch_rate_raw);
		}

		if (sent) {
			_last_gimbal_tx = hrt_absolute_time();
			++_gimbal_transmissions;
		}

		return sent;
	}
	bool write_all(const uint8_t *data, size_t length)
	{
		size_t sent = 0;
		const hrt_abstime deadline = hrt_absolute_time() + 200000;

		while (sent < length && !should_exit() && hrt_absolute_time() < deadline) {
			const ssize_t count = ::write(_fd, data + sent, length - sent);

			if (count > 0) { sent += count; }

			else if (count < 0 && errno != EAGAIN && errno != EINTR) { break; }

			else { px4_usleep(500); }
		}

		return sent == length;
	}
	bool send_zhongke_command(uint8_t code, int16_t param_x = 0, int16_t param_y = 0)
	{
		// Vendor doc 表1 控制指令数据: 16-byte frame, 0xEB 0x90 sync,
		// command in byte 2, little-endian X/Y in bytes 3..6, checksum over bytes 0..14.
		uint8_t frame[16] {};
		frame[0] = 0xEB;
		frame[1] = 0x90;
		frame[2] = code;
		write_s16(&frame[3], param_x);
		write_s16(&frame[5], param_y);
		uint32_t checksum = 0;

		for (unsigned i = 0; i < 15; ++i) { checksum += frame[i]; }

		frame[15] = checksum & 0xFF;

		if (!write_all(frame, sizeof(frame))) { PX4_ERR("zhongke command TX failed"); return false; }

		return true;
	}
	bool send_aisheng_command(uint8_t code, int16_t param1 = 0, int16_t param2 = 0)
	{
		// Vendor doc 表1: 45-byte control frame, 0xAA55 header, length 0x2D,
		// command in byte 4 and little-endian parameters 1/2 in bytes 5..8
		// (document numbering). Other parameters/attitude/position stay zero, the
		// timestamp (bytes 41-44) carries the flight controller uptime in ms.
		// The vendor recommends sending each command as three consecutive frames.
		const uint32_t timestamp_ms = hrt_absolute_time() / 1000;

		for (unsigned repeat = 0; repeat < 3; ++repeat) {
			uint8_t frame[45] {};
			frame[0] = 0xAA;
			frame[1] = 0x55;
			frame[2] = 0x2D;
			frame[3] = code;
			write_s16(&frame[4], param1);
			write_s16(&frame[6], param2);
			frame[40] = timestamp_ms & 0xFF;
			frame[41] = (timestamp_ms >> 8) & 0xFF;
			frame[42] = (timestamp_ms >> 16) & 0xFF;
			frame[43] = (timestamp_ms >> 24) & 0xFF;
			uint32_t checksum = 0;

			for (unsigned i = 3; i <= 43; ++i) { checksum += frame[i]; }

			frame[44] = checksum & 0xFF;

			if (!write_all(frame, sizeof(frame))) { PX4_ERR("aisheng command TX failed"); return false; }
		}

		return true;
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

		for (unsigned i = 0; i < 5; ++i) {
			const uint8_t *target = &_buffer[28 + i * 12];
			msg.target_type[i] = target[0];
			msg.target_confidence[i] = target[1] * 0.01f;
			msg.target_lon[i] = read_s32(&target[2]);
			msg.target_lat[i] = read_s32(&target[6]);
			msg.target_alt[i] = read_u16(&target[10]) * 0.1f;
		}

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
	int _trace_fd{-1};
	int _gimbal_fd{-1};
	uint8_t _brand{kBrandZhongke};
	hrt_abstime _start_time{0};
	hrt_abstime _command_interval_us{100000};
	hrt_abstime _last_gimbal_tx{0};
	hrt_abstime _last_slew_time{0};
	float _rate_max{10.f};
	float _rate_slew{20.f};
	float _aisheng_gain{10.f};
	float _target_yaw_rate{0.f};
	float _target_pitch_rate{0.f};
	float _current_yaw_rate{0.f};
	float _current_pitch_rate{0.f};
	bool _gimbal_active{false};
	uint8_t _buffer[192] {};
	unsigned _length{0};
	uint32_t _zhongke_frames{0};
	uint32_t _zhongke_checksum_errors{0};
	uint32_t _aisheng_frames{0};
	uint32_t _aisheng_checksum_errors{0};
	uint32_t _aisheng_length_errors{0};
	uint32_t _noise_bytes{0};
	uint64_t _total_bytes{0};
	uint32_t _trace_starts{0};
	uint32_t _trace_stops{0};
	uint32_t _gimbal_commands{0};
	uint32_t _gimbal_stops{0};
	uint32_t _gimbal_transmissions{0};
	uORB::Publication<dyt_zhongke_telemetry_s> _zhongke_pub{ORB_ID(dyt_zhongke_telemetry)};
	uORB::Publication<dyt_aisheng_telemetry_s> _aisheng_pub{ORB_ID(dyt_aisheng_telemetry)};
	uORB::Publication<dyt_trace_command_s> _trace_pub{ORB_ID(dyt_trace_command)};
	uORB::Publication<dyt_gimbal_rate_command_s> _gimbal_pub{ORB_ID(dyt_gimbal_rate_command)};
};

extern "C" __EXPORT int serial_probe_main(int argc, char *argv[])
{
	return SerialProbe::main(argc, argv);
}
