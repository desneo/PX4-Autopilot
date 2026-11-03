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

#include "AishengJsonParser.hpp"

namespace
{
constexpr uint16_t kDefaultPort = 9000;
using aisheng_udp::kUdpBufferSize;
}

class AishengUdpBridge : public ModuleBase<AishengUdpBridge>
{
public:
	explicit AishengUdpBridge(uint16_t port) : _port(port) {}

	~AishengUdpBridge() override { close_socket(); }

	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("aisheng_udp_bridge", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
					      8192, (px4_main_t)&run_trampoline, (char *const *)argv);

		if (_task_id < 0) { return PX4_ERROR; }

		return PX4_OK;
	}

	static AishengUdpBridge *instantiate(int argc, char *argv[])
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
						PX4_ERR("无效 UDP 端口: %s", myoptarg);
						return nullptr;
					}
				}
				break;

			default:
				return nullptr;
			}
		}

		return new AishengUdpBridge(static_cast<uint16_t>(port));
	}

	static int custom_command(int argc, char *argv[]) { return print_usage("未知命令"); }

	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_DESCRIPTION("Receive complete ASD-2-075-DD0 JSON telemetry and publish dyt_aisheng_telemetry in SITL.");
		PRINT_MODULE_USAGE_NAME("aisheng_udp_bridge", "simulation");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_PARAM_INT('p', kDefaultPort, 1, 65535, "UDP listen port", true);
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return PX4_OK;
	}

	int print_status() override
	{
		PX4_INFO("UDP: %s 0.0.0.0:%u 接收=%" PRIu32 " 有效=%" PRIu32 " 解析错误=%" PRIu32
			 " 超长=%" PRIu32 " 发布=%" PRIu32,
			 _fd >= 0 ? "监听中" : "不可用", _port, _datagrams, _valid,
			 _parse_errors, _oversize, _publications);

		if (_last_rx != 0) {
			const hrt_abstime age = hrt_elapsed_time(&_last_rx);
			PX4_INFO("UDP 对端=%s:%u 更新间隔=%" PRIu64 " ms", _last_peer, _last_peer_port,
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

				PX4_ERR("轮询失败: %d", errno);
				break;
			}

			if (fdset.revents & (POLLERR | POLLHUP | POLLNVAL)) {
				PX4_ERR("UDP socket 失败");
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
			PX4_ERR("创建 UDP socket 失败: %d", errno);
			return false;
		}

		// 独占监听端口，防止与两轴导引头桥同时接收同一端口。
		const int flags = fcntl(_fd, F_GETFL, 0);

		if (flags < 0 || fcntl(_fd, F_SETFL, flags | O_NONBLOCK) != 0) {
			PX4_ERR("设置 UDP 非阻塞模式失败: %d", errno);
			close_socket();
			return false;
		}

		sockaddr_in address{};
		address.sin_family = AF_INET;
		address.sin_addr.s_addr = htonl(INADDR_ANY);
		address.sin_port = htons(_port);

		if (bind(_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
			PX4_ERR("绑定 UDP 端口 %u 失败: %d", _port, errno);
			close_socket();
			return false;
		}

		PX4_INFO("完整遥测接收已就绪 UDP 0.0.0.0:%u", _port);
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
					PX4_ERR("接收 UDP 数据失败: %d", errno);
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
			dyt_aisheng_telemetry_s message{};
			aisheng_udp::JsonParser parser(buffer, static_cast<size_t>(count));

			if (!parser.parse(message)) {
				++_parse_errors;
				continue;
			}

			++_valid;
			inet_ntop(AF_INET, &peer.sin_addr, _last_peer, sizeof(_last_peer));
			_last_peer_port = ntohs(peer.sin_port);
			publish(message);
		}
	}

	void publish(dyt_aisheng_telemetry_s &message)
	{
		// 转发时间和设备毫秒时间均不属于 PX4 启动时钟。
		message.timestamp = hrt_absolute_time();
		message.los_omega_n = NAN;
		message.los_omega_e = NAN;
		message.los_omega_d = NAN;

		if (_publisher.publish(message)) {
			++_publications;
			_last_rx = message.timestamp;

		} else {
			PX4_ERR("发布 dyt_aisheng_telemetry 失败");
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

extern "C" __EXPORT int aisheng_udp_bridge_main(int argc, char *argv[])
{
	return AishengUdpBridge::main(argc, argv);
}
