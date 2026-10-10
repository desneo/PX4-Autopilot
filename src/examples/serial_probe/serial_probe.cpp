#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/serial_probe.h>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SerialProbe : public ModuleBase<SerialProbe>
{
public:
	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("serial_probe", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
					      2600, (px4_main_t)&run_trampoline, (char *const *)argv);

		if (_task_id < 0) { return PX4_ERROR; }

		return wait_until_running();
	}
	static SerialProbe *instantiate(int argc, char *argv[]) { return new SerialProbe(); }
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_DESCRIPTION("TELEM2 115200 8N1: PING,sequence,value -> uORB -> value+1 -> PONG");
		PRINT_MODULE_USAGE_NAME("serial_probe", "example");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}
	int print_status() override
	{
		PX4_INFO("%s 115200; inspect serial_probe_request/response with listener", _device);
		return 0;
	}
	void run() override
	{
		_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) { PX4_ERR("open: %d", errno); return; }

		termios cfg{};

		if (tcgetattr(_fd, &cfg) != 0) { PX4_ERR("tcgetattr"); close_port(); return; }

		cfmakeraw(&cfg);
		cfg.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
		cfg.c_cflag |= CS8 | CLOCAL | CREAD;
		cfsetispeed(&cfg, B115200);
		cfsetospeed(&cfg, B115200);

		if (tcsetattr(_fd, TCSANOW, &cfg) != 0) { PX4_ERR("tcsetattr"); close_port(); return; }

		tcflush(_fd, TCIOFLUSH);
		_response_fd = orb_subscribe(ORB_ID(serial_probe_response));

		if (_response_fd < 0) { PX4_ERR("subscribe response failed"); close_port(); return; }

		PX4_INFO("ready on %s at 115200", _device);

		while (!should_exit()) {
			px4_pollfd_struct_t fdset[2] {};
			fdset[0].fd = _fd;
			fdset[0].events = POLLIN;
			fdset[1].fd = _response_fd;
			fdset[1].events = POLLIN;
			const int ready = px4_poll(fdset, 2, 50);

			if (ready < 0) {
				if (errno == EINTR) { continue; }

				PX4_ERR("poll: %d", errno); break;
			}

			if (fdset[0].revents & (POLLERR | POLLHUP | POLLNVAL)) { PX4_ERR("serial disconnected"); break; }

			// The independent processor wakes this task by publishing uORB responses.
			if (fdset[1].revents & POLLIN) { send_responses(); }

			if (fdset[0].revents & POLLIN) {
				char bytes[128];
				const ssize_t count = ::read(_fd, bytes, sizeof(bytes));

				for (ssize_t i = 0; i < count; ++i) { receive(bytes[i]); }
			}
		}

		close_port();
	}
private:
	void close_port()
	{
		if (_response_fd >= 0) { orb_unsubscribe(_response_fd); _response_fd = -1; }

		if (_fd >= 0) { ::close(_fd); _fd = -1; }
	}
	void send_line(const char *line)
	{
		size_t sent = 0;
		const size_t length = strlen(line);
		const hrt_abstime deadline = hrt_absolute_time() + 100000;

		while (sent < length && !should_exit() && hrt_absolute_time() < deadline) {
			const ssize_t count = ::write(_fd, line + sent, length - sent);

			if (count > 0) { sent += count; }

			else if (count < 0 && errno != EAGAIN && errno != EINTR) { break; }

			else { px4_usleep(1000); }
		}

		if (sent != length) { PX4_ERR("TX incomplete"); }
	}
	void receive(char byte)
	{
		if (byte == '\n') {
			if (_overflow) { send_line("ERR,LENGTH\n"); }

			else {
				if (_length > 0 && _line[_length - 1] == '\r') { --_length; }

				_line[_length] = '\0';
				handle_line();
			}

			_length = 0;
			_overflow = false;

		} else if (!_overflow) {
			if (_length + 1 < sizeof(_line)) { _line[_length++] = byte; }

			else { _overflow = true; }
		}
	}
	void handle_line()
	{
		// Only decimal digits for sequence; value may have a leading minus sign.
		bool valid = _length >= 8 && memcmp(_line, "PING,", 5) == 0;
		char *separator = valid ? strchr(_line + 5, ',') : nullptr;
		valid = valid && separator && separator > _line + 5 && separator[1] != '\0';

		if (valid) {
			for (const char *p = _line + 5; p < separator; ++p) { valid &= *p >= '0' && *p <= '9'; }

			const char *start = separator + 1;

			if (*start == '-') { ++start; }

			valid &= *start != '\0';

			for (const char *p = start; p < _line + _length; ++p) { valid &= *p >= '0' && *p <= '9'; }
		}

		unsigned long sequence = 0;
		long value = 0;

		if (valid) {
			errno = 0;
			sequence = strtoul(_line + 5, nullptr, 10);
			valid &= errno == 0 && sequence <= UINT32_MAX;
			errno = 0;
			value = strtol(separator + 1, nullptr, 10);
			valid &= errno == 0 && value >= INT32_MIN && value < INT32_MAX;
		}

		if (!valid) { send_line("ERR,FORMAT\n"); return; }

		serial_probe_s request{};
		request.timestamp = hrt_absolute_time();
		request.sequence = static_cast<uint32_t>(sequence);
		request.value = static_cast<int32_t>(value);

		if (!_request_pub.publish(request)) { PX4_ERR("publish request failed"); return; }

	}
	void send_responses()
	{
		while (!should_exit()) {
			bool updated = false;

			if (orb_check(_response_fd, &updated) != PX4_OK || !updated) { break; }

			serial_probe_s response{};

			if (orb_copy(ORB_ID(serial_probe_response), _response_fd, &response) != PX4_OK) {
				PX4_ERR("copy response failed"); break;
			}

			char reply[64];
			snprintf(reply, sizeof(reply), "PONG,%lu,%ld\n", (unsigned long)response.sequence, (long)response.value);
			send_line(reply);
		}
	}
	char _device[16] {"/dev/ttyS3"};
	int _fd{-1};
	int _response_fd{-1};
	char _line[64] {};
	size_t _length{0};
	bool _overflow{false};
	uORB::Publication<serial_probe_s> _request_pub{ORB_ID(serial_probe_request)};
};

extern "C" __EXPORT int serial_probe_main(int argc, char *argv[])
{
	return SerialProbe::main(argc, argv);
}
