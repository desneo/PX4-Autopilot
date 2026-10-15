#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="${TMPDIR:-/tmp}/px4-scenarios/01_common"

collect_pids() {
	{
		pgrep -f '[/]build/px4_sitl[^/]*/bin/px4' || true
		pgrep -f '(^|[ /])gz sim([ ]|$)' || true
		pgrep -f '[t]est_mavsdk_mission[.]sh' || true
		pgrep -f '[f]ly_mission.*udpout://[^ ]+:1428[0-9]' || true
		pgrep -f '[m]avsdk_mission.*udpout://[^ ]+:1428[0-9]' || true
	} | sort -nu
}

mapfile -t pids < <(collect_pids)
if ((${#pids[@]} == 0)); then
	echo "No PX4 SITL environment is running."
	rm -rf -- "${RUNTIME_DIR}"
	exit 0
fi

echo "Stopping all current PX4 SITL/Gazebo processes: ${pids[*]}"
kill -TERM "${pids[@]}" 2>/dev/null || true

for _ in {1..50}; do
	mapfile -t remaining < <(collect_pids)
	((${#remaining[@]} == 0)) && break
	sleep 0.1
done

mapfile -t remaining < <(collect_pids)
if ((${#remaining[@]} > 0)); then
	echo "Force-stopping remaining processes: ${remaining[*]}"
	kill -KILL "${remaining[@]}" 2>/dev/null || true
fi

rm -rf -- "${RUNTIME_DIR}"
echo "All PX4 SITL environments have been stopped."
