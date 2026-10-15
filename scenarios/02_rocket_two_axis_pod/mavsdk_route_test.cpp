// Native MAVSDK route-flight integration test for Scenario 02.
//
// The route is generated around the vehicle's current GPS position so the
// test is independent of the world's geographic origin and each vehicle's
// spawn offset.

#if __has_include(<mavsdk/mavsdk.hpp>)
#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/mission/mission.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>
#else
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

using namespace mavsdk;
using namespace std::chrono_literals;

namespace {

constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr float kRouteAltitudeM = 15.0F;
constexpr float kRouteSpeedMps = 5.0F;
constexpr double kRouteEdgeM = 20.0;
constexpr double kMinimumHorizontalTravelM = 12.0;
constexpr float kMinimumAltitudeM = 8.0F;

double radians(double degrees)
{
	return degrees * kPi / 180.0;
}

double horizontal_distance_m(
	double latitude_a_deg, double longitude_a_deg,
	double latitude_b_deg, double longitude_b_deg)
{
	const double latitude_a = radians(latitude_a_deg);
	const double latitude_b = radians(latitude_b_deg);
	const double delta_latitude = latitude_b - latitude_a;
	const double delta_longitude = radians(longitude_b_deg - longitude_a_deg);
	const double sin_latitude = std::sin(delta_latitude / 2.0);
	const double sin_longitude = std::sin(delta_longitude / 2.0);
	const double a = sin_latitude * sin_latitude
		+ std::cos(latitude_a) * std::cos(latitude_b) * sin_longitude * sin_longitude;
	return 2.0 * kEarthRadiusM * std::asin(std::sqrt(std::min(1.0, a)));
}

std::pair<double, double> offset_coordinate(
	double origin_latitude_deg, double origin_longitude_deg,
	double north_m, double east_m)
{
	const double latitude = origin_latitude_deg + north_m / kEarthRadiusM * 180.0 / kPi;
	const double longitude = origin_longitude_deg
		+ east_m / (kEarthRadiusM * std::cos(radians(origin_latitude_deg))) * 180.0 / kPi;
	return {latitude, longitude};
}

Mission::MissionItem make_item(double latitude_deg, double longitude_deg, bool fly_through)
{
	Mission::MissionItem item{};
	item.latitude_deg = latitude_deg;
	item.longitude_deg = longitude_deg;
	item.relative_altitude_m = kRouteAltitudeM;
	item.speed_m_s = kRouteSpeedMps;
	item.is_fly_through = fly_through;
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

bool valid_position(const Telemetry::Position &position)
{
	return std::isfinite(position.latitude_deg) && std::isfinite(position.longitude_deg)
		&& std::abs(position.latitude_deg) <= 90.0 && std::abs(position.longitude_deg) <= 180.0
		&& (std::abs(position.latitude_deg) > 0.001 || std::abs(position.longitude_deg) > 0.001);
}

} // namespace

int main(int argc, char **argv)
{
	if (argc != 2) {
		std::cerr << "Usage: " << argv[0] << " udpout://SITL_HOST:PORT\n";
		return 2;
	}

	Mavsdk sdk{Mavsdk::Configuration{ComponentType::GroundStation}};
	const auto connection_result = sdk.add_any_connection(argv[1]);

	if (connection_result != ConnectionResult::Success) {
		std::cerr << "Connection failed: " << connection_result << '\n';
		return 1;
	}

	const auto system = sdk.first_autopilot(30.0);

	if (!system) {
		std::cerr << "Timed out waiting for an autopilot\n";
		return 1;
	}

	Action action{system.value()};
	Mission mission{system.value()};
	Telemetry telemetry{system.value()};

	if (!wait_until([&telemetry]() { return telemetry.health_all_ok(); }, 120s)) {
		std::cerr << "Timed out waiting for vehicle health\n";
		return 1;
	}

	Telemetry::Position start_position{};
	if (!wait_until(
		[&telemetry, &start_position]() {
			start_position = telemetry.position();
			return valid_position(start_position);
		},
		30s)) {
		std::cerr << "Timed out waiting for a valid global position\n";
		return 1;
	}

	const std::vector<std::pair<double, double>> local_route_m{
		{0.0, 0.0},
		{kRouteEdgeM, 0.0},
		{kRouteEdgeM, kRouteEdgeM},
		{0.0, kRouteEdgeM},
		{0.0, 0.0},
	};

	Mission::MissionPlan plan{};
	for (std::size_t index = 0; index < local_route_m.size(); ++index) {
		const auto coordinate = offset_coordinate(
			start_position.latitude_deg, start_position.longitude_deg,
			local_route_m[index].first, local_route_m[index].second);
		plan.mission_items.push_back(make_item(
			coordinate.first, coordinate.second, index + 1 < local_route_m.size()));
	}

	std::cout << std::fixed << std::setprecision(8)
		<< "Start position: " << start_position.latitude_deg << ", "
		<< start_position.longitude_deg << '\n'
		<< "Uploading " << plan.mission_items.size() << " waypoints; route edge="
		<< kRouteEdgeM << " m, altitude=" << kRouteAltitudeM << " m\n";

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

	auto recover_vehicle = [&action, &telemetry]() {
		if (!telemetry.armed()) {
			return;
		}

		std::cerr << "Requesting return-to-launch after test failure\n";
		if (action.return_to_launch() != Action::Result::Success) {
			(void)action.land();
		}
		(void)wait_until([&telemetry]() { return !telemetry.armed(); }, 120s);
	};

	std::atomic<int> current{-1};
	std::atomic<int> total{-1};
	const auto progress_handle = mission.subscribe_mission_progress(
		[&current, &total](Mission::MissionProgress progress) {
			current = progress.current;
			total = progress.total;
			std::cout << "Mission progress: " << progress.current << " / " << progress.total << '\n';
		});

	if (const auto result = mission.start_mission(); result != Mission::Result::Success) {
		mission.unsubscribe_mission_progress(progress_handle);
		std::cerr << "Starting mission failed: " << result << '\n';
		recover_vehicle();
		return 1;
	}

	double maximum_horizontal_distance_m = 0.0;
	float maximum_relative_altitude_m = 0.0F;
	bool mission_finished = false;
	const auto mission_deadline = std::chrono::steady_clock::now() + 240s;

	while (std::chrono::steady_clock::now() < mission_deadline) {
		const auto position = telemetry.position();
		if (valid_position(position)) {
			maximum_horizontal_distance_m = std::max(
				maximum_horizontal_distance_m,
				horizontal_distance_m(
					start_position.latitude_deg, start_position.longitude_deg,
					position.latitude_deg, position.longitude_deg));
			maximum_relative_altitude_m = std::max(
				maximum_relative_altitude_m, position.relative_altitude_m);
		}

		const auto result = mission.is_mission_finished();
		if (result.first == Mission::Result::Success && result.second) {
			mission_finished = true;
			break;
		}

		std::this_thread::sleep_for(1s);
	}

	mission.unsubscribe_mission_progress(progress_handle);
	if (!mission_finished) {
		std::cerr << "Timed out during mission at " << current << " / " << total << '\n';
		recover_vehicle();
		return 1;
	}

	if (const auto result = action.return_to_launch(); result != Action::Result::Success) {
		std::cerr << "Return-to-launch failed: " << result << '\n';
		recover_vehicle();
		return 1;
	}

	if (!wait_until([&telemetry]() { return !telemetry.armed(); }, 180s)) {
		std::cerr << "Timed out waiting for landing/disarm\n";
		recover_vehicle();
		return 1;
	}

	std::cout << std::setprecision(2)
		<< "Observed maximum horizontal distance: " << maximum_horizontal_distance_m << " m\n"
		<< "Observed maximum relative altitude: " << maximum_relative_altitude_m << " m\n";

	if (maximum_horizontal_distance_m < kMinimumHorizontalTravelM) {
		std::cerr << "Route validation failed: aircraft did not travel far enough\n";
		return 1;
	}

	if (maximum_relative_altitude_m < kMinimumAltitudeM) {
		std::cerr << "Route validation failed: aircraft did not climb high enough\n";
		return 1;
	}

	std::cout << "PASS: route flown, mission completed, vehicle returned and disarmed\n";
	return 0;
}
