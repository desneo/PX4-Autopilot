/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/dyt_aisheng_telemetry.h>

#include <arpa/inet.h>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr uint16_t kDefaultPort = 9000;
constexpr size_t kUdpBufferSize = 512;
constexpr uint16_t kRequiredFields = (1u << 9) - 1u;

enum class Field : uint8_t {
	TrackStatus = 0,
	PitchAngle,
	YawAngle,
	PitchMisalignment,
	YawMisalignment,
	TrackingConfidence,
	TargetConfidence,
	PitchLosRate,
	YawLosRate,
};

struct AishengFields {
	uint8_t track_status{0};
	float pitch_angle{0.f};
	float yaw_angle{0.f};
	float pitch_misalignment{0.f};
	float yaw_misalignment{0.f};
	float tracking_confidence{0.f};
	float target_confidence{0.f};
	float pitch_los_rate{0.f};
	float yaw_los_rate{0.f};
};

class JsonParser
{
public:
	JsonParser(const char *data, size_t length) : _cursor(data), _end(data + length) {}

	bool parse(AishengFields &fields)
	{
		skip_whitespace();

		if (!consume('{')) { return false; }

		skip_whitespace();

		if (consume('}')) { return false; }

		uint16_t seen = 0;

		while (_cursor < _end) {
			char key[32] {};

			if (!parse_string(key, sizeof(key))) { return false; }

			skip_whitespace();

			if (!consume(':')) { return false; }

			skip_whitespace();
			const int field = field_index(key);

			if (field >= 0) {
				const uint16_t bit = static_cast<uint16_t>(1u << field);

				if ((seen & bit) != 0) { return false; }

				float value = 0.f;

				if (!parse_number(value) || !assign_field(static_cast<Field>(field), value, fields)) {
					return false;
				}

				seen |= bit;

			} else if (!skip_value(0)) {
				return false;
			}

			skip_whitespace();

			if (consume('}')) { break; }

			if (!consume(',')) { return false; }

			skip_whitespace();
		}

		skip_whitespace();
		return _cursor == _end && seen == kRequiredFields;
	}

private:
	static bool is_whitespace(char value)
	{
		return value == ' ' || value == '\t' || value == '\r' || value == '\n';
	}

	static bool is_digit(char value) { return value >= '0' && value <= '9'; }

	void skip_whitespace()
	{
		while (_cursor < _end && is_whitespace(*_cursor)) { ++_cursor; }
	}

	bool consume(char expected)
	{
		if (_cursor >= _end || *_cursor != expected) { return false; }

		++_cursor;
		return true;
	}

	bool consume_literal(const char *literal)
	{
		const size_t length = strlen(literal);

		if (static_cast<size_t>(_end - _cursor) < length || strncmp(_cursor, literal, length) != 0) {
			return false;
		}

		_cursor += length;
		return true;
	}

	bool parse_string(char *output, size_t output_size)
	{
		if (!consume('"')) { return false; }

		size_t written = 0;
		bool overflow = false;

		while (_cursor < _end) {
			char value = *_cursor++;

			if (value == '"') {
				if (output != nullptr && output_size > 0) {
					if (overflow) {
						output[0] = '\0';

					} else {
						output[written] = '\0';
					}
				}

				return true;
			}

			if (static_cast<unsigned char>(value) < 0x20) { return false; }

			if (value == '\\') {
				if (_cursor >= _end) { return false; }

				const char escaped = *_cursor++;

				switch (escaped) {
				case '"': value = '"'; break;
				case '\\': value = '\\'; break;
				case '/': value = '/'; break;
				case 'b': value = '\b'; break;
				case 'f': value = '\f'; break;
				case 'n': value = '\n'; break;
				case 'r': value = '\r'; break;
				case 't': value = '\t'; break;

				case 'u':
					for (unsigned i = 0; i < 4; ++i) {
						if (_cursor >= _end
						    || !(is_digit(*_cursor) || (*_cursor >= 'a' && *_cursor <= 'f')
							 || (*_cursor >= 'A' && *_cursor <= 'F'))) {
							return false;
						}

						++_cursor;
					}

					value = '?';
					break;

				default: return false;
				}
			}

			if (output != nullptr && output_size > 0) {
				if (written + 1 < output_size) {
					output[written++] = value;

				} else {
					overflow = true;
				}
			}
		}

		return false;
	}

	bool parse_number(float &value)
	{
		const char *start = _cursor;

		if (_cursor < _end && *_cursor == '-') { ++_cursor; }

		if (_cursor >= _end) { return false; }

		if (*_cursor == '0') {
			++_cursor;

			if (_cursor < _end && is_digit(*_cursor)) { return false; }

		} else if (*_cursor >= '1' && *_cursor <= '9') {
			while (_cursor < _end && is_digit(*_cursor)) { ++_cursor; }

		} else {
			return false;
		}

		if (_cursor < _end && *_cursor == '.') {
			++_cursor;

			if (_cursor >= _end || !is_digit(*_cursor)) { return false; }

			while (_cursor < _end && is_digit(*_cursor)) { ++_cursor; }
		}

		if (_cursor < _end && (*_cursor == 'e' || *_cursor == 'E')) {
			++_cursor;

			if (_cursor < _end && (*_cursor == '+' || *_cursor == '-')) { ++_cursor; }

			if (_cursor >= _end || !is_digit(*_cursor)) { return false; }

			while (_cursor < _end && is_digit(*_cursor)) { ++_cursor; }
		}

		char *conversion_end = nullptr;
		value = strtof(start, &conversion_end);
		return conversion_end == _cursor && std::isfinite(value);
	}

	bool skip_value(unsigned depth)
	{
		if (depth > 8 || _cursor >= _end) { return false; }

		if (*_cursor == '"') { return parse_string(nullptr, 0); }

		if (*_cursor == '{') {
			++_cursor;
			skip_whitespace();

			if (consume('}')) { return true; }

			while (_cursor < _end) {
				if (!parse_string(nullptr, 0)) { return false; }

				skip_whitespace();

				if (!consume(':')) { return false; }

				skip_whitespace();

				if (!skip_value(depth + 1)) { return false; }

				skip_whitespace();

				if (consume('}')) { return true; }

				if (!consume(',')) { return false; }

				skip_whitespace();
			}

			return false;
		}

		if (*_cursor == '[') {
			++_cursor;
			skip_whitespace();

			if (consume(']')) { return true; }

			while (_cursor < _end) {
				if (!skip_value(depth + 1)) { return false; }

				skip_whitespace();

				if (consume(']')) { return true; }

				if (!consume(',')) { return false; }

				skip_whitespace();
			}

			return false;
		}

		if (consume_literal("true") || consume_literal("false") || consume_literal("null")) { return true; }

		float unused = 0.f;
		return parse_number(unused);
	}

	static int field_index(const char *key)
	{
		if (strcmp(key, "track_status") == 0) { return static_cast<int>(Field::TrackStatus); }

		if (strcmp(key, "pitch_angle") == 0) { return static_cast<int>(Field::PitchAngle); }

		if (strcmp(key, "yaw_angle") == 0) { return static_cast<int>(Field::YawAngle); }

		if (strcmp(key, "pitch_misalignment") == 0) { return static_cast<int>(Field::PitchMisalignment); }

		if (strcmp(key, "yaw_misalignment") == 0) { return static_cast<int>(Field::YawMisalignment); }

		if (strcmp(key, "tracking_confidence") == 0) { return static_cast<int>(Field::TrackingConfidence); }

		if (strcmp(key, "target_confidence") == 0) { return static_cast<int>(Field::TargetConfidence); }

		if (strcmp(key, "pitch_los_rate") == 0) { return static_cast<int>(Field::PitchLosRate); }

		if (strcmp(key, "yaw_los_rate") == 0) { return static_cast<int>(Field::YawLosRate); }

		return -1;
	}

	static bool assign_field(Field field, float value, AishengFields &fields)
	{
		switch (field) {
		case Field::TrackStatus:
			if (fabsf(value) < 0.000001f) {
				fields.track_status = 0;

			} else if (fabsf(value - 1.f) < 0.000001f) {
				fields.track_status = 1;

			} else {
				return false;
			}

			break;

		case Field::PitchAngle: fields.pitch_angle = value; break;
		case Field::YawAngle: fields.yaw_angle = value; break;
		case Field::PitchMisalignment: fields.pitch_misalignment = value; break;
		case Field::YawMisalignment: fields.yaw_misalignment = value; break;
		case Field::TrackingConfidence: fields.tracking_confidence = value; break;
		case Field::TargetConfidence: fields.target_confidence = value; break;
		case Field::PitchLosRate: fields.pitch_los_rate = value; break;
		case Field::YawLosRate: fields.yaw_los_rate = value; break;
		}

		return true;
	}

	const char *_cursor;
	const char *_end;
};
}

class DytUdpBridge : public ModuleBase<DytUdpBridge>
{
public:
	explicit DytUdpBridge(uint16_t port) : _port(port) {}

	~DytUdpBridge() override { close_socket(); }

	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("dyt_udp_bridge", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
					      4096, (px4_main_t)&run_trampoline, (char *const *)argv);

		if (_task_id < 0) { return PX4_ERROR; }

		return PX4_OK;
	}

	static DytUdpBridge *instantiate(int argc, char *argv[])
	{
		long port = kDefaultPort;
		int myoptind = 1;
		int ch = 0;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "p:", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'p': {
				char *end = nullptr;
				errno = 0;
				port = strtol(myoptarg, &end, 10);

				if (errno != 0 || end == myoptarg || *end != '\0' || port < 1 || port > 65535) {
					PX4_ERR("invalid UDP port: %s", myoptarg);
					return nullptr;
				}
			}
			break;

			default:
				return nullptr;
			}
		}

		return new DytUdpBridge(static_cast<uint16_t>(port));
	}

	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_DESCRIPTION("Receive Aisheng JSON telemetry over UDP and publish dyt_aisheng_telemetry in SITL.");
		PRINT_MODULE_USAGE_NAME("dyt_udp_bridge", "simulation");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_PARAM_INT('p', kDefaultPort, 1, 65535, "UDP listen port", true);
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return PX4_OK;
	}

	int print_status() override
	{
		PX4_INFO("udp: %s 0.0.0.0:%u rx=%" PRIu32 " ok=%" PRIu32 " parse_err=%" PRIu32
			 " oversize=%" PRIu32 " pub=%" PRIu32,
			 _fd >= 0 ? "listening" : "unavailable", _port, _datagrams, _valid,
			 _parse_errors, _oversize, _publications);

		if (_last_rx != 0) {
			const hrt_abstime age = hrt_elapsed_time(&_last_rx);
			PX4_INFO("udp peer=%s:%u age=%" PRIu64 " ms", _last_peer, _last_peer_port,
				 static_cast<uint64_t>(age / 1000));
		}

		return PX4_OK;
	}

	void run() override
	{
		if (!open_socket()) { return; }

		while (!should_exit()) {
			pollfd fdset{};
			fdset.fd = _fd;
			fdset.events = POLLIN;
			const int ready = ::poll(&fdset, 1, 100);

			if (ready < 0) {
				if (errno == EINTR) { continue; }

				PX4_ERR("poll failed: %d", errno);
				break;
			}

			if (fdset.revents & (POLLERR | POLLHUP | POLLNVAL)) {
				PX4_ERR("UDP socket failed");
				break;
			}

			if (fdset.revents & POLLIN) { receive(); }
		}

		close_socket();
	}

private:
	bool open_socket()
	{
		_fd = socket(AF_INET, SOCK_DGRAM, 0);

		if (_fd < 0) {
			PX4_ERR("create UDP socket failed: %d", errno);
			return false;
		}

		const int reuse = 1;

		if (setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) != 0) {
			PX4_WARN("SO_REUSEADDR failed: %d", errno);
		}

		const int flags = fcntl(_fd, F_GETFL, 0);

		if (flags >= 0) { (void)fcntl(_fd, F_SETFL, flags | O_NONBLOCK); }

		sockaddr_in address{};
		address.sin_family = AF_INET;
		address.sin_addr.s_addr = htonl(INADDR_ANY);
		address.sin_port = htons(_port);

		if (bind(_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
			PX4_ERR("bind UDP port %u failed: %d", _port, errno);
			close_socket();
			return false;
		}

		PX4_INFO("listening for Aisheng JSON on UDP 0.0.0.0:%u", _port);
		return true;
	}

	void close_socket()
	{
		if (_fd >= 0) {
			::close(_fd);
			_fd = -1;
		}
	}

	void receive()
	{
		while (_fd >= 0 && !should_exit()) {
			char buffer[kUdpBufferSize] {};
			sockaddr_in peer{};
			socklen_t peer_length = sizeof(peer);
			const ssize_t count = recvfrom(_fd, buffer, sizeof(buffer) - 1, MSG_TRUNC,
						       reinterpret_cast<sockaddr *>(&peer), &peer_length);

			if (count < 0) {
				if (errno != EAGAIN
#if EWOULDBLOCK != EAGAIN
				    && errno != EWOULDBLOCK
#endif
				    && errno != EINTR) {
					PX4_ERR("recvfrom failed: %d", errno);
				}

				return;
			}

			++_datagrams;

			if (count <= 0) {
				++_parse_errors;
				continue;
			}

			if (static_cast<size_t>(count) >= sizeof(buffer)) {
				++_oversize;
				continue;
			}

			buffer[count] = '\0';
			AishengFields fields{};
			JsonParser parser(buffer, static_cast<size_t>(count));

			if (!parser.parse(fields)) {
				++_parse_errors;
				continue;
			}

			++_valid;
			inet_ntop(AF_INET, &peer.sin_addr, _last_peer, sizeof(_last_peer));
			_last_peer_port = ntohs(peer.sin_port);
			publish(fields);
		}
	}

	void publish(const AishengFields &fields)
	{
		dyt_aisheng_telemetry_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.track_status = fields.track_status;
		msg.pitch_angle = fields.pitch_angle;
		msg.yaw_angle = fields.yaw_angle;
		msg.pitch_misalignment = fields.pitch_misalignment;
		msg.yaw_misalignment = fields.yaw_misalignment;
		msg.tracking_confidence = fields.tracking_confidence;
		msg.target_confidence[0] = fields.target_confidence;
		msg.pitch_los_rate = fields.pitch_los_rate;
		msg.yaw_los_rate = fields.yaw_los_rate;

		if (_publisher.publish(msg)) {
			++_publications;
			_last_rx = msg.timestamp;

		} else {
			PX4_ERR("publish dyt_aisheng_telemetry failed");
		}
	}

	uint16_t _port{kDefaultPort};
	int _fd{-1};
	hrt_abstime _last_rx{0};
	char _last_peer[INET_ADDRSTRLEN] {"none"};
	uint16_t _last_peer_port{0};
	uint32_t _datagrams{0};
	uint32_t _valid{0};
	uint32_t _parse_errors{0};
	uint32_t _oversize{0};
	uint32_t _publications{0};
	uORB::Publication<dyt_aisheng_telemetry_s> _publisher{ORB_ID(dyt_aisheng_telemetry)};
};

extern "C" __EXPORT int dyt_udp_bridge_main(int argc, char *argv[])
{
	return DytUdpBridge::main(argc, argv);
}
