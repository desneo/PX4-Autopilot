#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/serial_probe.h>
#include <cerrno>
#include <climits>

class SerialProbeProcessor : public ModuleBase<SerialProbeProcessor>
{
public:
	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("serial_proc", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT,
					      2200, (px4_main_t)&run_trampoline, (char *const *)argv);

		if (_task_id < 0) { return PX4_ERROR; }

		return wait_until_running();
	}
	static SerialProbeProcessor *instantiate(int argc, char *argv[]) { return new SerialProbeProcessor(); }
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr)
	{
		if (reason) { PX4_WARN("%s", reason); }

		PRINT_MODULE_DESCRIPTION("Independent uORB request processor: publish value+1 responses.");
		PRINT_MODULE_USAGE_NAME("serial_probe_processor", "example");
		PRINT_MODULE_USAGE_COMMAND("start");
		PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
		return 0;
	}
	int print_status() override
	{
		PX4_INFO("independent task: serial_probe_request -> value+1 -> serial_probe_response");
		return 0;
	}
	void run() override
	{
		const int request_fd = orb_subscribe(ORB_ID(serial_probe_request));

		if (request_fd < 0) { PX4_ERR("subscribe request failed"); return; }

		PX4_INFO("waiting for uORB requests");

		while (!should_exit()) {
			px4_pollfd_struct_t fdset{};
			fdset.fd = request_fd;
			fdset.events = POLLIN;
			const int ready = px4_poll(&fdset, 1, 100);

			if (ready < 0) {
				if (errno == EINTR) { continue; }

				PX4_ERR("request poll: %d", errno); break;
			}

			if (fdset.revents & (POLLERR | POLLHUP | POLLNVAL)) { PX4_ERR("request poll failed"); break; }

			if (fdset.revents & POLLIN) {
				// Drain queued requests instead of keeping only the latest value.
				while (!should_exit()) {
					bool updated = false;

					if (orb_check(request_fd, &updated) != PX4_OK || !updated) { break; }

					serial_probe_s request{};

					if (orb_copy(ORB_ID(serial_probe_request), request_fd, &request) != PX4_OK) {
						PX4_ERR("copy request failed"); break;
					}

					if (request.value == INT32_MAX) { PX4_ERR("request would overflow"); continue; }

					serial_probe_s response{};
					response.timestamp = hrt_absolute_time();
					response.sequence = request.sequence;
					response.value = request.value + 1;

					if (!_response_pub.publish(response)) { PX4_ERR("publish response failed"); }
				}
			}
		}

		orb_unsubscribe(request_fd);
	}
private:
	uORB::Publication<serial_probe_s> _response_pub{ORB_ID(serial_probe_response)};
};

extern "C" __EXPORT int serial_probe_processor_main(int argc, char *argv[])
{
	return SerialProbeProcessor::main(argc, argv);
}
