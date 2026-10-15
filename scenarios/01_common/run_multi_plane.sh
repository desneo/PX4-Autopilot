#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PX4_DIR=$(cd -- "${SCRIPT_DIR}/../.." && pwd)
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
RUNTIME_DIR="${TMPDIR:-/tmp}/px4-scenarios/01_common"
COUNT=4
GUI=0
WORLD=default
PARTITION=px4_scenario_01_common

usage() {
	cat <<'EOF'
Usage: run_multi_plane.sh [-n COUNT] [--gui]

Start COUNT gz_x500 vehicles in one Gazebo world (default: 4).
Gazebo GUI is disabled by default. MAVSDK connects to UDP 14280+i.

Options:
  -n, --count COUNT  Number of vehicles, 1-10 (default: 4)
      --gui          Start the Gazebo GUI
  -h, --help         Show this help
EOF
}

while (($# > 0)); do
	case "$1" in
		-n|--count)
			(($# >= 2)) || { echo "ERROR: $1 requires a value" >&2; exit 2; }
			COUNT=$2
			shift 2
			;;
		--gui)
			GUI=1
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			echo "ERROR: unknown argument: $1" >&2
			usage >&2
			exit 2
			;;
	esac
done

if [[ ! ${COUNT} =~ ^[0-9]+$ ]] || ((COUNT < 1 || COUNT > 10)); then
	echo "ERROR: COUNT must be an integer from 1 to 10" >&2
	exit 2
fi

if pgrep -f '[/]build/px4_sitl[^/]*/bin/px4' >/dev/null \
	|| pgrep -f '(^|[ /])gz sim([ ]|$)' >/dev/null; then
	echo "ERROR: a PX4 SITL or Gazebo simulation is already running." >&2
	echo "Run ${SCRIPT_DIR}/stop.sh before starting this scenario." >&2
	exit 1
fi

if [[ ! -x ${PX4_BIN} ]]; then
	echo "PX4 SITL is not built; building px4_sitl_default first."
	make -C "${PX4_DIR}" px4_sitl_default
fi

rm -rf -- "${RUNTIME_DIR}"
mkdir -p -- "${RUNTIME_DIR}"
printf '%s\n' "${COUNT}" > "${RUNTIME_DIR}/count"

columns=1
while ((columns * columns < COUNT)); do
	((columns += 1))
done

wait_until_ready() {
	local pid=$1
	local log_file=$2
	local instance=$3

	for _ in {1..45}; do
		if grep -q 'Startup script returned successfully' "${log_file}" 2>/dev/null; then
			return 0
		fi
		if ! kill -0 "${pid}" 2>/dev/null; then
			echo "ERROR: PX4 instance ${instance} exited during startup" >&2
			tail -n 40 "${log_file}" >&2 || true
			"${SCRIPT_DIR}/stop.sh" >/dev/null 2>&1 || true
			exit 1
		fi
		sleep 1
	done

	echo "ERROR: PX4 instance ${instance} did not become ready in 45 seconds" >&2
	tail -n 40 "${log_file}" >&2 || true
	"${SCRIPT_DIR}/stop.sh" >/dev/null 2>&1 || true
	exit 1
}

echo "Starting ${COUNT} gz_x500 vehicle(s) in Gazebo world '${WORLD}'."
echo "PX4 MAVSDK UDP ports: 14280-$((14280 + COUNT - 1))"

for ((i = 0; i < COUNT; i++)); do
	x=$(((i % columns) * 5))
	y=$(((i / columns) * 5))
	log_file="${RUNTIME_DIR}/px4_${i}.log"
	local_port=$((14280 + i))

	env_args=(
		"GZ_PARTITION=${PARTITION}"
		"GZ_IP=127.0.0.1"
		"PX4_SIM_MODEL=gz_x500"
		"PX4_GZ_WORLD=${WORLD}"
		"PX4_GZ_MODEL_POSE=${x},${y},0,0,0,0"
	)
	((GUI == 0)) && env_args+=("HEADLESS=1")
	((i > 0)) && env_args+=("PX4_GZ_STANDALONE=1")

	printf 'PX4[%d] command: ' "${i}"
	printf '%q ' nohup env "${env_args[@]}" "${PX4_BIN}" -d -i "${i}"
	printf '> %q 2>&1 < /dev/null &\n' "${log_file}"

	(
		cd "${PX4_DIR}"
		nohup env "${env_args[@]}" "${PX4_BIN}" -d -i "${i}" \
			>"${log_file}" 2>&1 </dev/null &
		echo $! > "${RUNTIME_DIR}/px4_${i}.pid"
	)
	pid=$(<"${RUNTIME_DIR}/px4_${i}.pid")
	wait_until_ready "${pid}" "${log_file}" "${i}"
	echo "PX4[${i}] ready: model=x500_${i}, pose=(${x}, ${y}), MAVSDK=udpout://<SITL_HOST>:${local_port}"
done

echo
echo "Scenario is ready. Logs: ${RUNTIME_DIR}"
echo "Run MAVSDK route test: ${SCRIPT_DIR}/test_mavsdk_mission.sh -n ${COUNT}"
echo "Stop all PX4 SITL:       ${SCRIPT_DIR}/stop.sh"
