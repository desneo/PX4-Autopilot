// Minimal native MAVSDK route-flight test used by Scenario 01.
//
// This source intentionally targets the MAVSDK 3.x C++ headers archived in
// /home/firefly/code/3rdparty/common.  Building this application does not
// rebuild or modify MAVSDK.

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

using namespace mavsdk;
using namespace std::chrono_literals;

namespace {

Mission::MissionItem make_item(double latitude_deg, double longitude_deg)
{
	Mission::MissionItem item{};
	item.latitude_deg = latitude_deg;
	item.longitude_deg = longitude_deg;
	item.relative_altitude_m = 10.0f;
	item.speed_m_s = 5.0f;
	item.is_fly_through = true;
	item.camera_action = Mission::MissionItem::CameraAction::None;
	return item;
}

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::seconds timeout)
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;

	while (std::chrono::steady_clock::now() < deadline) {
		if (predicate()) {
			return true;
		}

		std::this_thread::sleep_for(1s);
	}

	return false;
}

} // namespace

int main(int argc, char **argv)
{
	if (argc != 2) {
		std::cerr << "Usage: " << argv[0] << " udpin://0.0.0.0:PORT\n";
		return 2;
	}

	Mavsdk sdk{Mavsdk::Configuration{ComponentType::GroundStation}};
	const auto connection_result = sdk.add_any_connection(argv[1]);

	if (connection_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << connection_result << '\n';
		return 1;
	}

	const auto system = sdk.first_autopilot(20.0);

	if (!system) {
		std::cerr << "Timed out waiting for an autopilot\n";
		return 1;
	}

	Action action{system.value()};
	Mission mission{system.value()};
	Telemetry telemetry{system.value()};

	if (!wait_until([&telemetry]() { return telemetry.health_all_ok(); }, 90s)) {
		std::cerr << "Timed out waiting for vehicle health\n";
		return 1;
	}

	Mission::MissionPlan plan{};
	plan.mission_items = {
		make_item(47.398170327054473, 8.545649021863966),
		make_item(47.398241338125118, 8.545536011457443),
		make_item(47.398139363821485, 8.545384615659714),
		make_item(47.398058617228855, 8.545461803674698),
	};

	std::cout << "Uploading " << plan.mission_items.size() << " waypoints\n";

	if (const auto result = mission.upload_mission(plan); result != Mission::Result::Success) {
		std::cerr << "Mission upload failed: " << result << '\n';
		return 1;
	}

	if (const auto result = action.hold(); result != Action::Result::Success) {
		std::cerr << "Switching to Hold failed: " << result << '\n';
		return 1;
	}

	if (const auto result = action.arm(); result != Action::Result::Success) {
		std::cerr << "Arming failed: " << result << '\n';
		return 1;
	}

	std::atomic<int> current{-1};
	std::atomic<int> total{-1};
	const auto progress_handle = mission.subscribe_mission_progress(
		[&current, &total](Mission::MissionProgress progress) {
			current = progress.current;
			total = progress.total;
			std::cout << "Mission progress: " << progress.current << " / " << progress.total << '\n';
		});

	if (const auto result = mission.start_mission(); result != Mission::Result::Success) {
		std::cerr << "Starting mission failed: " << result << '\n';
		return 1;
	}

	if (!wait_until(
			[&mission]() {
				const auto result = mission.is_mission_finished();
				return result.first == Mission::Result::Success && result.second;
			},
			180s)) {
		std::cerr << "Timed out during mission at " << current << " / " << total << '\n';
		return 1;
	}

	mission.unsubscribe_mission_progress(progress_handle);

	if (const auto result = action.return_to_launch(); result != Action::Result::Success) {
		std::cerr << "Return-to-launch failed: " << result << '\n';
		return 1;
	}

	std::this_thread::sleep_for(2s);

	if (!wait_until([&telemetry]() { return !telemetry.armed(); }, 120s)) {
		std::cerr << "Timed out waiting for landing/disarm\n";
		return 1;
	}

	std::cout << "PASS: mission completed, vehicle returned and disarmed\n";
	return 0;
}
